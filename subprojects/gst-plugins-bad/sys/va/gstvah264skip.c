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

/* A picture whose every macroblock is P_Skip repeats its reference
 * exactly: with refIdxL0 0 and no available or non-zero neighbour
 * motion, the predicted motion of every skipped macroblock is zero
 * (8.4.1.1), and with no residual and equal motion the deblocking
 * strength is zero everywhere. Building it on the CPU costs a few
 * hundred bytes of bit writing, where submitting it to the encoder
 * costs a full hardware frame including the rate control firmware. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstvah264skip.h"

#include <gst/base/gstbitwriter.h>
#include <gst/codecparsers/gsth264bitwriter.h>

#include <string.h>

/* Table 9-44 and the LPS column of Table 9-45 of ITU-T H.264. */
static const guint8 range_tab_lps[64][4] = {
  {128, 176, 208, 240}, {128, 167, 197, 227},
  {128, 158, 187, 216}, {123, 150, 178, 205},
  {116, 142, 169, 195}, {111, 135, 160, 185},
  {105, 128, 152, 175}, {100, 122, 144, 166},
  { 95, 116, 137, 158}, { 90, 110, 130, 150},
  { 85, 104, 123, 142}, { 81,  99, 117, 135},
  { 77,  94, 111, 128}, { 73,  89, 105, 122},
  { 69,  85, 100, 116}, { 66,  80,  95, 110},
  { 62,  76,  90, 104}, { 59,  72,  86,  99},
  { 56,  69,  81,  94}, { 53,  65,  77,  89},
  { 51,  62,  73,  85}, { 48,  59,  69,  80},
  { 46,  56,  66,  76}, { 43,  53,  63,  72},
  { 41,  50,  59,  69}, { 39,  48,  56,  65},
  { 37,  45,  54,  62}, { 35,  43,  51,  59},
  { 33,  41,  48,  56}, { 32,  39,  46,  53},
  { 30,  37,  43,  50}, { 29,  35,  41,  48},
  { 27,  33,  39,  45}, { 26,  31,  37,  43},
  { 24,  30,  35,  41}, { 23,  28,  33,  39},
  { 22,  27,  32,  37}, { 21,  26,  30,  35},
  { 20,  24,  29,  33}, { 19,  23,  27,  31},
  { 18,  22,  26,  30}, { 17,  21,  25,  28},
  { 16,  20,  23,  27}, { 15,  19,  22,  25},
  { 14,  18,  21,  24}, { 14,  17,  20,  23},
  { 13,  16,  19,  22}, { 12,  15,  18,  21},
  { 12,  14,  17,  20}, { 11,  14,  16,  19},
  { 11,  13,  15,  18}, { 10,  12,  15,  17},
  { 10,  12,  14,  16}, {  9,  11,  13,  15},
  {  9,  11,  12,  14}, {  8,  10,  12,  14},
  {  8,   9,  11,  13}, {  7,   9,  11,  12},
  {  7,   9,  10,  12}, {  7,   8,  10,  11},
  {  6,   8,   9,  11}, {  6,   7,   9,  10},
  {  6,   7,   8,   9}, {  2,   2,   2,   2},
};

static const guint8 trans_idx_lps[64] = {
   0,  0,  1,  2,  2,  4,  4,  5,  6,  7,  8,  9,  9, 11, 11, 12,
  13, 13, 15, 15, 16, 16, 18, 18, 19, 19, 21, 21, 22, 22, 23, 24,
  24, 25, 26, 26, 27, 27, 28, 29, 29, 30, 30, 30, 31, 32, 32, 33,
  33, 33, 34, 34, 35, 35, 35, 36, 36, 36, 37, 37, 37, 38, 38, 63,
};
typedef struct
{
  GstBitWriter *bw;
  guint low;
  guint range;
  guint outstanding;
  gboolean first_bit;
  gboolean ok;
} CabacEncoder;

static void
_cabac_put_bit (CabacEncoder * c, guint8 bit)
{
  if (c->first_bit)
    c->first_bit = FALSE;
  else
    c->ok &= gst_bit_writer_put_bits_uint8 (c->bw, bit, 1);

  for (; c->outstanding > 0; c->outstanding--)
    c->ok &= gst_bit_writer_put_bits_uint8 (c->bw, 1 - bit, 1);
}

/* 9.3.4.2, RenormE */
static void
_cabac_renorm (CabacEncoder * c)
{
  while (c->range < 256) {
    if (c->low < 256) {
      _cabac_put_bit (c, 0);
    } else if (c->low >= 512) {
      c->low -= 512;
      _cabac_put_bit (c, 1);
    } else {
      c->low -= 256;
      c->outstanding++;
    }
    c->range <<= 1;
    c->low <<= 1;
  }
}

/* 9.3.4.2, EncodeDecision */
static void
_cabac_encode_decision (CabacEncoder * c, guint8 * state, guint8 * mps,
    guint8 bin)
{
  guint lps = range_tab_lps[*state][(c->range >> 6) & 3];

  c->range -= lps;
  if (bin != *mps) {
    c->low += c->range;
    c->range = lps;
    if (*state == 0)
      *mps = 1 - *mps;
    *state = trans_idx_lps[*state];
  } else if (*state < 62) {
    (*state)++;
  }

  _cabac_renorm (c);
}

/* 9.3.4.5, EncodeTerminate, with EncodeFlush when bin is 1. The last
 * bit the flush writes is the rbsp_stop_one_bit. */
static void
_cabac_encode_terminate (CabacEncoder * c, guint8 bin)
{
  c->range -= 2;
  if (!bin) {
    _cabac_renorm (c);
    return;
  }

  c->low += c->range;
  c->range = 2;
  _cabac_renorm (c);
  _cabac_put_bit (c, (c->low >> 9) & 1);
  c->ok &= gst_bit_writer_put_bits_uint8 (c->bw, ((c->low >> 7) & 3) | 1, 2);
}

static gboolean
_write_cabac_skip_data (GstBitWriter * bw, const GstH264SliceHdr * slice_hdr,
    guint num_mbs)
{
  /* Table 9-13, ctxIdx 11 (mb_skip_flag of a P slice) for each
   * cabac_init_idc. Every neighbour is skipped or unavailable, so
   * ctxIdxInc is always 0 and ctxIdx 11 is the only context used. */
  static const gint8 init_mn[3][2] = { {23, 33}, {22, 25}, {29, 16} };
  CabacEncoder c = { bw, 0, 510, 0, TRUE, TRUE };
  gint slice_qp, pre_state;
  guint8 state, mps;
  guint i;

  if (slice_hdr->cabac_init_idc > 2)
    return FALSE;

  /* cabac_alignment_one_bit */
  while (gst_bit_writer_get_size (bw) % 8)
    c.ok &= gst_bit_writer_put_bits_uint8 (bw, 1, 1);

  slice_qp = 26 + slice_hdr->pps->pic_init_qp_minus26 +
      slice_hdr->slice_qp_delta;
  pre_state = ((init_mn[slice_hdr->cabac_init_idc][0] *
          CLAMP (slice_qp, 0, 51)) >> 4) +
      init_mn[slice_hdr->cabac_init_idc][1];
  pre_state = CLAMP (pre_state, 1, 126);
  if (pre_state <= 63) {
    state = 63 - pre_state;
    mps = 0;
  } else {
    state = pre_state - 64;
    mps = 1;
  }

  for (i = 0; i < num_mbs; i++) {
    _cabac_encode_decision (&c, &state, &mps, 1);
    _cabac_encode_terminate (&c, i == num_mbs - 1);
  }

  /* rbsp_alignment_zero_bit */
  while (gst_bit_writer_get_size (bw) % 8)
    c.ok &= gst_bit_writer_put_bits_uint8 (bw, 0, 1);

  return c.ok;
}

static gboolean
_write_ue (GstBitWriter * bw, guint32 value)
{
  guint32 v = value + 1;
  guint len = g_bit_storage (v);

  return gst_bit_writer_put_bits_uint32 (bw, 0, len - 1) &&
      gst_bit_writer_put_bits_uint32 (bw, v, len);
}

/**
 * gst_va_h264_skip_picture_new:
 * @slice_hdr: the slice header of the picture, a P slice whose
 *   reference list 0 starts with the picture to repeat
 * @num_mbs: the macroblocks in the picture
 *
 * Builds a non-reference picture made of one slice in which every
 * macroblock is P_Skip, as an Annex B NAL with its start code.
 *
 * Returns: (transfer full) (nullable): the coded picture
 */
GstBuffer *
gst_va_h264_skip_picture_new (const GstH264SliceHdr * slice_hdr, guint num_mbs)
{
  GstH264SliceHdr hdr_copy;
  guint8 hdr[4 + 256] = { 0, };
  guint i;
  guint hdr_size = sizeof (hdr), trail_bits = 0;
  GstBitWriter bw;
  guint8 *raw = NULL, *nal = NULL;
  guint raw_bits, nal_size;
  gboolean ok;

  g_return_val_if_fail (slice_hdr != NULL, NULL);
  g_return_val_if_fail (slice_hdr->pps != NULL, NULL);
  g_return_val_if_fail (GST_H264_IS_P_SLICE (slice_hdr), NULL);
  g_return_val_if_fail (num_mbs > 0, NULL);

  hdr_copy = *slice_hdr;
  if (slice_hdr->pps->weighted_pred_flag) {
    GstH264PredWeightTable *pwt = &hdr_copy.pred_weight_table;

    /* A zeroed table reads as an explicit weight of 0, which would
     * repeat the reference as black. The default weight writes no
     * explicit weight at all. */
    memset (pwt, 0, sizeof (*pwt));
    for (i = 0; i <= hdr_copy.num_ref_idx_l0_active_minus1 && i < 32; i++) {
      pwt->luma_weight_l0[i] = 1;
      pwt->chroma_weight_l0[i][0] = 1;
      pwt->chroma_weight_l0[i][1] = 1;
    }
  }

  if (gst_h264_bit_writer_slice_hdr (&hdr_copy, TRUE, GST_H264_NAL_SLICE,
          FALSE, hdr, &hdr_size, &trail_bits) != GST_H264_BIT_WRITER_OK)
    return NULL;

  gst_bit_writer_init (&bw);
  ok = gst_bit_writer_put_bytes (&bw, hdr, hdr_size);
  if (trail_bits)
    ok &= gst_bit_writer_put_bits_uint8 (&bw, hdr[hdr_size] >> (8 - trail_bits),
        trail_bits);

  if (slice_hdr->pps->entropy_coding_mode_flag) {
    ok &= _write_cabac_skip_data (&bw, slice_hdr, num_mbs);
  } else {
    /* mb_skip_run covering the picture, then rbsp_slice_trailing_bits */
    ok &= _write_ue (&bw, num_mbs);
    ok &= gst_bit_writer_put_bits_uint8 (&bw, 1, 1);
    while (gst_bit_writer_get_size (&bw) % 8)
      ok &= gst_bit_writer_put_bits_uint8 (&bw, 0, 1);
  }

  if (!ok) {
    gst_bit_writer_reset (&bw);
    return NULL;
  }

  raw_bits = gst_bit_writer_get_size (&bw);
  raw = gst_bit_writer_reset_and_get_data (&bw);

  /* Emulation prevention can add one byte per two input bytes at most. */
  nal_size = raw_bits / 8 + raw_bits / 16 + 8;
  nal = g_malloc (nal_size);
  if (gst_h264_bit_writer_convert_to_nal (4, FALSE, TRUE, FALSE, raw,
          raw_bits, nal, &nal_size) != GST_H264_BIT_WRITER_OK) {
    g_free (raw);
    g_free (nal);
    return NULL;
  }
  g_free (raw);

  return gst_buffer_new_wrapped (nal, nal_size);
}
