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

#pragma once

#include <glib.h>

G_BEGIN_DECLS

typedef enum
{
  GST_VA_RC_FRAME_I = 0,
  GST_VA_RC_FRAME_P,
  GST_VA_RC_FRAME_B,
  GST_VA_RC_FRAME_TYPES,
} GstVaRcFrameType;

/* The decision for one picture. The encoder keeps it with the picture
 * and hands it back with the coded size, in whatever order the coded
 * sizes become known. */
typedef struct
{
  gboolean valid;
  GstVaRcFrameType type;
  guint qp;
  gdouble predicted_bits;
  /* The P complexity the prediction used, to learn the ratio of the
   * other frame types against it. */
  gdouble log_complexity_p;
  guint intra_count;
} GstVaRcFrame;

typedef struct
{
  gdouble bitrate;
  gdouble fps;
  gdouble buffer_size;
  guint intra_period;
  guint min_qp;
  guint max_qp;
  guint divisor;
  guint initial_qp;

  /* Encoder buffer fullness in bits. Pictures whose coded size is not
   * known yet count with their predicted size. */
  gdouble fullness;
  /* log2 of c in bits = c * 2^(-qp / 6) for a P picture, and of the
   * cost of the other frame types relative to it. */
  gdouble log_complexity_p;
  gdouble log_ratio[GST_VA_RC_FRAME_TYPES];
  gboolean seen[GST_VA_RC_FRAME_TYPES];
  gint last_qp[GST_VA_RC_FRAME_TYPES];
  /* What the last I picture cost above a plain picture budget, paid
   * back over the intra period. */
  gdouble intra_excess;
  guint intra_count;
  guint ticks_since_intra;
  /* The last P picture was taken as a scene cut. */
  gboolean after_cut;
} GstVaRateControl;

void     gst_va_rate_control_init          (GstVaRateControl * rc,
                                            guint bitrate,
                                            guint fps_n,
                                            guint fps_d,
                                            guint buffer_size,
                                            guint intra_period,
                                            guint min_qp,
                                            guint max_qp,
                                            guint pixels);
void     gst_va_rate_control_set_bitrate   (GstVaRateControl * rc,
                                            guint bitrate,
                                            guint buffer_size);
void     gst_va_rate_control_set_qp_range  (GstVaRateControl * rc,
                                            guint min_qp,
                                            guint max_qp);
void     gst_va_rate_control_set_divisor   (GstVaRateControl * rc,
                                            guint divisor);
void     gst_va_rate_control_pick          (GstVaRateControl * rc,
                                            GstVaRcFrameType type,
                                            GstVaRcFrame * frame);
void     gst_va_rate_control_skip          (GstVaRateControl * rc,
                                            guint bits);
void     gst_va_rate_control_update        (GstVaRateControl * rc,
                                            const GstVaRcFrame * frame,
                                            guint bits);

G_END_DECLS
