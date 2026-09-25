/* GStreamer
 *  Copyright (C) 2026 irlenc
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

/* A constant bitrate controller that runs in the application, so the
 * hardware can encode in CQP mode. On VDEnc the driver's own CBR runs
 * the HuC rate control firmware twice per picture on the video engine,
 * a fixed cost that dominates small renditions; here the QP of every
 * picture is chosen on the CPU and reaches the hardware as the slice
 * QP delta.
 *
 * The state is a leaky bucket: every input frame period drains
 * bitrate / fps bits, every picture adds its coded size. The size of a
 * picture is modelled as bits = c * 2^(-qp / 6), with c learnt per
 * frame type by exponential smoothing in the log domain from the coded
 * sizes. A P picture gets the per picture budget, corrected towards the
 * planned fullness over half a second, and the QP the model says hits
 * it, moved at most a few steps from the last P picture. I and B
 * pictures follow the P QP at a fixed offset, so quality does not pump
 * at every intra period; the cost an I picture adds above the budget is
 * paid back evenly by the P pictures of its period. Whatever the
 * smoothing wants, a picture the buffer cannot take is coded coarser.
 *
 * Nothing here assumes that a picture's coded size is known before the
 * next one is decided: a decision adds the predicted size to the
 * fullness, and the coded size, whenever it arrives, replaces it. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstvaratecontrol.h"

#include <math.h>
#include <string.h>

/* H.264 and H.265 double the quantizer step every 6 QP, so the residual
 * bits roughly halve. */
#define QP_PER_OCTAVE 6.0
/* Where the fullness settles between pictures that cost the budget. */
#define TARGET_LEVEL 0.2
/* A predicted picture must leave this much of the buffer free, the
 * prediction can be off. */
#define OVERFLOW_LEVEL 0.9
/* A fullness error is paid back with this time constant. */
#define FEEDBACK_SECONDS 0.5
#define MIN_TARGET_FRACTION 0.1
/* Largest QP change between consecutive P pictures that the buffer
 * does not force. */
#define MAX_QP_STEP 3
#define INTRA_QP_OFFSET -2
#define BIDIR_QP_OFFSET 2
/* The initial guesses of what I and B pictures cost against a P
 * picture at the same QP, until the coded sizes tell. */
#define INTRA_RATIO 5.0
#define BIDIR_RATIO 0.5
/* Smoothing of the P complexity. A picture far from the prediction,
 * more than twice or less than half, is most likely new content and
 * moves the model faster. */
#define COMPLEXITY_ALPHA 0.25
#define COMPLEXITY_ALPHA_JUMP 0.5
#define RATIO_ALPHA 0.5
/* A P picture within this many octaves of the cost of an I picture is
 * taken as a scene cut. */
#define CUT_MARGIN 0.5
/* A picture that costs about this many bits per pixel at the budget
 * codes at REFERENCE_QP. Only the first pictures use it. */
#define REFERENCE_BPP 0.05
#define REFERENCE_QP 30.0

static gint
_qp_for_bits (gdouble log_complexity, gdouble bits)
{
  return (gint) floor (QP_PER_OCTAVE * (log_complexity - log2 (bits)) + 0.5);
}

static gdouble
_predict_bits (gdouble log_complexity, gint qp)
{
  return exp2 (log_complexity - qp / QP_PER_OCTAVE);
}

static gdouble
_drain (const GstVaRateControl * rc)
{
  return rc->bitrate / rc->fps;
}

static void
_add_bits (GstVaRateControl * rc, gdouble bits)
{
  rc->fullness = MAX (0.0, rc->fullness + bits);
}

/* The bits the next P picture should cost: its share of the bitrate,
 * less its share of the last I picture's excess, corrected towards the
 * fullness planned at this point of the intra period. */
static gdouble
_target_bits (const GstVaRateControl * rc)
{
  gdouble budget = _drain (rc) * rc->divisor;
  gdouble level = rc->buffer_size * TARGET_LEVEL;
  gdouble horizon = MAX (1.0, rc->fps * FEEDBACK_SECONDS);
  gdouble target = budget;

  if (rc->intra_period > 1 && rc->ticks_since_intra < rc->intra_period) {
    gdouble left = 1.0 - (gdouble) rc->ticks_since_intra / rc->intra_period;

    level += rc->intra_excess * left;
    target -= rc->intra_excess * rc->divisor / rc->intra_period;
  }

  target += (level - rc->fullness) * rc->divisor / horizon;

  return MAX (target, budget * MIN_TARGET_FRACTION);
}

void
gst_va_rate_control_init (GstVaRateControl * rc, guint bitrate, guint fps_n,
    guint fps_d, guint buffer_size, guint intra_period, guint min_qp,
    guint max_qp, guint pixels)
{
  gdouble bpp;
  gint qp;
  guint i;

  g_return_if_fail (bitrate > 0 && fps_n > 0 && fps_d > 0 && pixels > 0);

  memset (rc, 0, sizeof (*rc));
  rc->bitrate = bitrate;
  rc->fps = (gdouble) fps_n / fps_d;
  rc->intra_period = intra_period;
  rc->divisor = 1;

  /* The HRD sets the buffer when there is one; a second of the bitrate
   * otherwise. */
  rc->buffer_size = buffer_size > 0 ? buffer_size : bitrate;
  gst_va_rate_control_set_qp_range (rc, min_qp, max_qp);

  bpp = _drain (rc) / pixels;
  qp = (gint) floor (REFERENCE_QP - QP_PER_OCTAVE * log2 (bpp / REFERENCE_BPP)
      + 0.5);
  rc->initial_qp = CLAMP (qp, (gint) rc->min_qp, (gint) rc->max_qp);

  rc->fullness = rc->buffer_size * TARGET_LEVEL;
  rc->log_complexity_p = log2 (_drain (rc)) + rc->initial_qp / QP_PER_OCTAVE;
  rc->log_ratio[GST_VA_RC_FRAME_I] = log2 (INTRA_RATIO);
  rc->log_ratio[GST_VA_RC_FRAME_P] = 0.0;
  rc->log_ratio[GST_VA_RC_FRAME_B] = log2 (BIDIR_RATIO);
  for (i = 0; i < GST_VA_RC_FRAME_TYPES; i++)
    rc->last_qp[i] = -1;
}

void
gst_va_rate_control_set_bitrate (GstVaRateControl * rc, guint bitrate,
    guint buffer_size)
{
  gdouble new_size;

  g_return_if_fail (bitrate > 0);

  new_size = buffer_size > 0 ? buffer_size : bitrate;

  /* Keep the buffer as full, relatively, as it was. The complexity
   * model does not depend on the bitrate, so the next picture already
   * gets the QP of the new target, within the QP step. */
  rc->fullness *= new_size / rc->buffer_size;
  rc->intra_excess *= new_size / rc->buffer_size;
  rc->buffer_size = new_size;
  rc->bitrate = bitrate;
}

void
gst_va_rate_control_set_qp_range (GstVaRateControl * rc, guint min_qp,
    guint max_qp)
{
  rc->max_qp = MIN (max_qp, 51);
  rc->min_qp = MIN (min_qp, rc->max_qp);
}

/* The skip pictures of repeated input never reach the controller as
 * pictures, see gst_va_rate_control_skip(). The divisor is the share
 * of input frames that are coded, so each coded picture gets the
 * budget of the frame periods it stands for. */
void
gst_va_rate_control_set_divisor (GstVaRateControl * rc, guint divisor)
{
  rc->divisor = MAX (divisor, 1);
}

void
gst_va_rate_control_pick (GstVaRateControl * rc, GstVaRcFrameType type,
    GstVaRcFrame * frame)
{
  gdouble log_complexity = rc->log_complexity_p + rc->log_ratio[type];
  gdouble limit = rc->buffer_size * OVERFLOW_LEVEL;
  gdouble predicted;
  gint qp, anchor;

  g_return_if_fail (type < GST_VA_RC_FRAME_TYPES);

  if (type == GST_VA_RC_FRAME_P
      || (type == GST_VA_RC_FRAME_I && rc->intra_period <= 1)) {
    qp = _qp_for_bits (log_complexity, _target_bits (rc));
    if (rc->last_qp[type] >= 0) {
      qp = CLAMP (qp, rc->last_qp[type] - MAX_QP_STEP,
          rc->last_qp[type] + MAX_QP_STEP);
    }
  } else {
    anchor = rc->last_qp[GST_VA_RC_FRAME_P] >= 0 ?
        rc->last_qp[GST_VA_RC_FRAME_P] : (gint) rc->initial_qp;
    qp = anchor + (type == GST_VA_RC_FRAME_I ?
        INTRA_QP_OFFSET : BIDIR_QP_OFFSET);
  }

  qp = CLAMP (qp, (gint) rc->min_qp, (gint) rc->max_qp);
  while (qp < (gint) rc->max_qp
      && rc->fullness + _predict_bits (log_complexity, qp) > limit)
    qp++;

  predicted = _predict_bits (log_complexity, qp);

  if (type == GST_VA_RC_FRAME_I) {
    rc->intra_excess = MAX (0.0, predicted - _drain (rc) * rc->divisor);
    rc->intra_count++;
    rc->ticks_since_intra = 0;
  }

  frame->valid = TRUE;
  frame->type = type;
  frame->qp = qp;
  frame->predicted_bits = predicted;
  frame->log_complexity_p = rc->log_complexity_p;
  frame->intra_count = rc->intra_count;

  rc->last_qp[type] = qp;
  _add_bits (rc, predicted - _drain (rc));
  rc->ticks_since_intra++;
}

/* A frame period whose picture was not coded by the hardware, a
 * repeat coded as a skip picture of @bits. */
void
gst_va_rate_control_skip (GstVaRateControl * rc, guint bits)
{
  _add_bits (rc, bits - _drain (rc));
  rc->ticks_since_intra++;
}

void
gst_va_rate_control_update (GstVaRateControl * rc, const GstVaRcFrame * frame,
    guint bits)
{
  gdouble observed, error, alpha;

  if (!frame->valid)
    return;

  _add_bits (rc, bits - frame->predicted_bits);

  if (frame->type == GST_VA_RC_FRAME_I && frame->intra_count == rc->intra_count) {
    rc->intra_excess = MAX (0.0, rc->intra_excess + bits -
        frame->predicted_bits);
  }

  observed = log2 (MAX (bits, 1)) + frame->qp / QP_PER_OCTAVE;

  if (frame->type == GST_VA_RC_FRAME_P || !rc->seen[GST_VA_RC_FRAME_P]) {
    /* Until a P picture is coded, the other types teach the P
     * complexity through their assumed ratio. */
    observed -= rc->log_ratio[frame->type];
    error = observed - rc->log_complexity_p;
    if (!rc->seen[frame->type]) {
      alpha = 1.0;
    } else if (frame->type == GST_VA_RC_FRAME_P && !rc->after_cut
        && error > rc->log_ratio[GST_VA_RC_FRAME_I] - CUT_MARGIN) {
      /* A P picture that cost about what an I picture would is most
       * likely a scene cut, coded mostly intra. The pictures after it
       * predict from the new scene, so they cost its intra cost less
       * the intra ratio. Only once in a row: a second such picture is
       * content that really got that much harder. */
      error -= rc->log_ratio[GST_VA_RC_FRAME_I];
      alpha = 1.0;
      rc->after_cut = TRUE;
    } else if (fabs (error) > 1.0) {
      alpha = COMPLEXITY_ALPHA_JUMP;
    } else {
      alpha = COMPLEXITY_ALPHA;
    }
    if (frame->type == GST_VA_RC_FRAME_P && alpha != 1.0)
      rc->after_cut = FALSE;
    rc->log_complexity_p += alpha * error;
  } else {
    observed -= frame->log_complexity_p;
    rc->log_ratio[frame->type] += RATIO_ALPHA *
        (observed - rc->log_ratio[frame->type]);
  }

  rc->seen[frame->type] = TRUE;
}
