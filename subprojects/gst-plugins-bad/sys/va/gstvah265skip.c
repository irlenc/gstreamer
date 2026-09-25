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

/* A P picture whose every coding unit is skipped with zero motion from
 * the first entry of reference list 0 repeats that picture exactly: no
 * residual and equal motion everywhere give every edge a boundary
 * strength of 0 (8.7.2.4), so deblocking changes nothing, and the slice
 * turns SAO off. Building it on the CPU costs a few hundred bytes of
 * bit writing, where submitting it to the encoder costs a full hardware
 * frame including the rate control firmware.
 *
 * Every CTU is one coding unit, split only where the picture boundary
 * forces it (7.3.8.4), with cu_skip_flag set. A skipped CU takes the
 * motion of a merge candidate (8.5.3.2.2), and the list starts with
 * the spatial candidates A1, B1, B0, A0, B2, then the temporal one,
 * then zero motion candidates. Every spatial neighbour inside the
 * picture is itself a skipped CU with zero motion on reference index
 * 0, so when one is available merge_idx 0 picks zero motion. When none
 * is (the first CU of the picture, or a neighbourhood hidden by the
 * parallel merge level), the list starts with the temporal candidate,
 * which copies whatever the collocated picture's motion was, followed
 * by zero candidates: merge_idx 1 picks zero motion whether or not the
 * temporal candidate exists. With one active reference every zero
 * candidate uses reference index 0.
 *
 * The temporal candidate is left enabled rather than turned off in the
 * slice: a picture with TemporalId 0 and slice_temporal_mvp_enabled_flag
 * 0 would forbid the pictures after it from using the motion of the
 * pictures before it (7.4.7.1), and a decoder may drop that motion
 * storage. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gstvah265skip.h"

#include <gst/base/gstbitwriter.h>
#include <gst/codecparsers/gsth265bitwriter.h>

#include <string.h>

/* rangeTabLps and transIdxLps of 9.3.4.3.2, the same tables as in
 * ITU-T H.264. */
/* *INDENT-OFF* */
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
/* *INDENT-ON* */

/* The contexts a skip picture codes, at their initValue for initType 1
 * and 2 (9.3.2.2): split_cu_flag (3), cu_transquant_bypass_flag,
 * cu_skip_flag (3) and the first bin of merge_idx. */
enum
{
  CTX_SPLIT_CU_FLAG = 0,
  CTX_CU_TRANSQUANT_BYPASS_FLAG = 3,
  CTX_CU_SKIP_FLAG = 4,
  CTX_MERGE_IDX = 7,
  NUM_CTX = 8,
};

static const guint8 init_values[2][NUM_CTX] = {
  {107, 139, 126, 154, 197, 185, 201, 122},
  {107, 139, 126, 154, 197, 185, 201, 137},
};

typedef struct
{
  guint8 state;
  guint8 mps;
} CabacContext;

typedef struct
{
  GstBitWriter *bw;
  guint low;
  guint range;
  guint outstanding;
  gboolean first_bit;
  gboolean ok;
} CabacEncoder;

typedef struct
{
  CabacEncoder c;
  CabacContext ctx[NUM_CTX];

  gint width;
  gint height;
  guint log2_min_cb;
  guint log2_par_mrg_level;
  gboolean transquant_bypass;
  gboolean temporal_mvp;

  /* CtDepth + 1 of the coding unit covering each minimum coding block
   * once it is coded, 0 before. In a single slice without tiles, coded
   * before is exactly the z-scan availability of 6.4.1. */
  guint8 *coded;
  guint coded_stride;
} SkipWriter;

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

/* 9.3.5.2, RenormE */
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

/* 9.3.5.2, EncodeDecision */
static void
_cabac_encode_decision (CabacEncoder * c, CabacContext * ctx, guint8 bin)
{
  guint lps = range_tab_lps[ctx->state][(c->range >> 6) & 3];

  c->range -= lps;
  if (bin != ctx->mps) {
    c->low += c->range;
    c->range = lps;
    if (ctx->state == 0)
      ctx->mps = 1 - ctx->mps;
    ctx->state = trans_idx_lps[ctx->state];
  } else if (ctx->state < 62) {
    ctx->state++;
  }

  _cabac_renorm (c);
}

/* 9.3.5.4, EncodeBypass */
static void
_cabac_encode_bypass (CabacEncoder * c, guint8 bin)
{
  c->low <<= 1;
  if (bin)
    c->low += c->range;

  if (c->low >= 1024) {
    _cabac_put_bit (c, 1);
    c->low -= 1024;
  } else if (c->low < 512) {
    _cabac_put_bit (c, 0);
  } else {
    c->low -= 512;
    c->outstanding++;
  }
}

/* 9.3.5.5, EncodeTerminate, with EncodeFlush when bin is 1. The last
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

/* 9.3.2.2 */
static void
_init_contexts (SkipWriter * w, guint init_type, gint slice_qp)
{
  gint qp = CLAMP (slice_qp, 0, 51);
  guint i;

  for (i = 0; i < NUM_CTX; i++) {
    guint8 init_value = init_values[init_type - 1][i];
    gint m = (init_value >> 4) * 5 - 45;
    gint n = ((init_value & 15) << 3) - 16;
    gint pre_state = CLAMP (((m * qp) >> 4) + n, 1, 126);

    if (pre_state <= 63) {
      w->ctx[i].state = 63 - pre_state;
      w->ctx[i].mps = 0;
    } else {
      w->ctx[i].state = pre_state - 64;
      w->ctx[i].mps = 1;
    }
  }
}

static guint
_coded_depth (SkipWriter * w, gint x, gint y)
{
  if (x < 0 || y < 0 || x >= w->width || y >= w->height)
    return 0;

  return w->coded[(y >> w->log2_min_cb) * w->coded_stride +
      (x >> w->log2_min_cb)];
}

/* A spatial merge candidate at (x_nb, y_nb) for the prediction block at
 * (x_pb, y_pb) exists when that block is coded (6.4.2; every coded
 * block here is inter) and outside the merge estimation region of the
 * prediction block (8.5.3.2.3). */
static gboolean
_merge_neighbour_available (SkipWriter * w, gint x_pb, gint y_pb,
    gint x_nb, gint y_nb)
{
  guint l = w->log2_par_mrg_level;

  if (!_coded_depth (w, x_nb, y_nb))
    return FALSE;

  return (x_pb >> l) != (x_nb >> l) || (y_pb >> l) != (y_nb >> l);
}

static gboolean
_has_spatial_merge_candidate (SkipWriter * w, gint x, gint y, gint size)
{
  /* A1, B1, B0, A0 and B2 of 8.5.3.2.3 */
  return _merge_neighbour_available (w, x, y, x - 1, y + size - 1)
      || _merge_neighbour_available (w, x, y, x + size - 1, y - 1)
      || _merge_neighbour_available (w, x, y, x + size, y - 1)
      || _merge_neighbour_available (w, x, y, x - 1, y + size)
      || _merge_neighbour_available (w, x, y, x - 1, y - 1);
}

/* 7.3.8.5, coding_unit () of a skipped CU */
static void
_write_skip_cu (SkipWriter * w, gint x0, gint y0, guint log2_size,
    guint depth)
{
  gint size = 1 << log2_size;
  guint ctx_inc;
  gint x, y;

  if (w->transquant_bypass)
    _cabac_encode_decision (&w->c, &w->ctx[CTX_CU_TRANSQUANT_BYPASS_FLAG], 0);

  /* 9.3.4.2.2: every coded neighbour is skipped */
  ctx_inc = (_coded_depth (w, x0 - 1, y0) != 0) +
      (_coded_depth (w, x0, y0 - 1) != 0);
  _cabac_encode_decision (&w->c, &w->ctx[CTX_CU_SKIP_FLAG + ctx_inc], 1);

  /* merge_idx, truncated rice with cMax MaxNumMergeCand - 1 = 4: the
   * first bin is context coded, the rest bypass. */
  if (w->temporal_mvp && !_has_spatial_merge_candidate (w, x0, y0, size)) {
    _cabac_encode_decision (&w->c, &w->ctx[CTX_MERGE_IDX], 1);
    _cabac_encode_bypass (&w->c, 0);
  } else {
    _cabac_encode_decision (&w->c, &w->ctx[CTX_MERGE_IDX], 0);
  }

  for (y = y0; y < y0 + size; y += 1 << w->log2_min_cb) {
    for (x = x0; x < x0 + size; x += 1 << w->log2_min_cb) {
      w->coded[(y >> w->log2_min_cb) * w->coded_stride +
          (x >> w->log2_min_cb)] = depth + 1;
    }
  }
}

/* 7.3.8.4, coding_quadtree () */
static void
_write_quadtree (SkipWriter * w, gint x0, gint y0, guint log2_size,
    guint depth)
{
  gint size = 1 << log2_size;
  gint half = size / 2;
  guint ctx_inc;

  if (x0 + size <= w->width && y0 + size <= w->height) {
    if (log2_size > w->log2_min_cb) {
      /* 9.3.4.2.2: condition is CtDepth of the neighbour > cqtDepth */
      ctx_inc = (_coded_depth (w, x0 - 1, y0) > depth + 1) +
          (_coded_depth (w, x0, y0 - 1) > depth + 1);
      _cabac_encode_decision (&w->c, &w->ctx[CTX_SPLIT_CU_FLAG + ctx_inc], 0);
    }

    _write_skip_cu (w, x0, y0, log2_size, depth);
    return;
  }

  /* split_cu_flag is not coded and inferred to be 1. The picture size
   * is a multiple of the minimum CB size, so this never happens at the
   * minimum size in a valid SPS. */
  if (log2_size <= w->log2_min_cb) {
    w->c.ok = FALSE;
    return;
  }

  _write_quadtree (w, x0, y0, log2_size - 1, depth + 1);
  if (x0 + half < w->width)
    _write_quadtree (w, x0 + half, y0, log2_size - 1, depth + 1);
  if (y0 + half < w->height)
    _write_quadtree (w, x0, y0 + half, log2_size - 1, depth + 1);
  if (x0 + half < w->width && y0 + half < w->height)
    _write_quadtree (w, x0 + half, y0 + half, log2_size - 1, depth + 1);
}

/* 7.3.8.1, slice_segment_data () and rbsp_slice_segment_trailing_bits ()
 * of a slice covering the picture, without tiles or wavefronts. */
static gboolean
_write_slice_data (GstBitWriter * bw, const GstH265SliceHdr * slice_hdr)
{
  const GstH265PPS *pps = slice_hdr->pps;
  const GstH265SPS *sps = pps->sps;
  guint log2_ctb = sps->log2_min_luma_coding_block_size_minus3 + 3 +
      sps->log2_diff_max_min_luma_coding_block_size;
  /* *INDENT-OFF* */
  SkipWriter w = {
    .c = { bw, 0, 510, 0, TRUE, TRUE },
    .width = sps->pic_width_in_luma_samples,
    .height = sps->pic_height_in_luma_samples,
    .log2_min_cb = sps->log2_min_luma_coding_block_size_minus3 + 3,
    .log2_par_mrg_level = pps->log2_parallel_merge_level_minus2 + 2,
    .transquant_bypass = pps->transquant_bypass_enabled_flag,
    .temporal_mvp = slice_hdr->temporal_mvp_enabled_flag,
  };
  /* *INDENT-ON* */
  guint width_ctbs, height_ctbs, i, init_type;

  if (w.width <= 0 || w.height <= 0 || log2_ctb > 6
      || w.width % (1 << w.log2_min_cb) || w.height % (1 << w.log2_min_cb))
    return FALSE;

  /* 9.3.2.2: initType of a P slice */
  init_type = slice_hdr->cabac_init_flag ? 2 : 1;
  _init_contexts (&w, init_type, 26 + pps->init_qp_minus26 +
      slice_hdr->qp_delta);

  w.coded_stride = w.width >> w.log2_min_cb;
  w.coded = g_malloc0 (w.coded_stride * (w.height >> w.log2_min_cb));

  width_ctbs = (w.width + (1 << log2_ctb) - 1) >> log2_ctb;
  height_ctbs = (w.height + (1 << log2_ctb) - 1) >> log2_ctb;

  for (i = 0; i < width_ctbs * height_ctbs && w.c.ok; i++) {
    _write_quadtree (&w, (i % width_ctbs) << log2_ctb,
        (i / width_ctbs) << log2_ctb, log2_ctb, 0);
    /* end_of_slice_segment_flag */
    _cabac_encode_terminate (&w.c, i == width_ctbs * height_ctbs - 1);
  }

  g_free (w.coded);

  /* rbsp_alignment_zero_bit */
  while (gst_bit_writer_get_size (bw) % 8)
    w.c.ok &= gst_bit_writer_put_bits_uint8 (bw, 0, 1);

  return w.c.ok;
}

/**
 * gst_va_h265_skip_picture_new:
 * @slice_hdr: the slice header of the picture. It provides the PPS
 *   (with its SPS), pic_order_cnt_lsb, an explicit short-term
 *   reference picture set whose first picture used by the current
 *   picture is the one to repeat, qp_delta and cabac_init_flag. Every
 *   other field is set here.
 *
 * Builds a sub-layer non-reference picture (TRAIL_N) made of one P
 * slice in which every coding unit is skipped with zero motion, as an
 * Annex B NAL with its start code. Tiles, wavefronts, separate colour
 * planes and screen content coding are not supported.
 *
 * Returns: (transfer full) (nullable): the coded picture
 */
GstBuffer *
gst_va_h265_skip_picture_new (const GstH265SliceHdr * slice_hdr)
{
  const GstH265PPS *pps;
  const GstH265SPS *sps;
  const GstH265ShortTermRefPicSet *rps;
  GstH265SliceHdr hdr;
  guint8 hdr_data[4 + 512] = { 0, };
  guint hdr_size = sizeof (hdr_data);
  GstBitWriter bw;
  guint8 *raw = NULL, *nal = NULL;
  guint raw_bits, nal_size, i;
  gint num_poc_total_curr = 0;
  gboolean ok;

  g_return_val_if_fail (slice_hdr != NULL, NULL);
  g_return_val_if_fail (slice_hdr->pps != NULL, NULL);
  g_return_val_if_fail (slice_hdr->pps->sps != NULL, NULL);
  g_return_val_if_fail (!slice_hdr->short_term_ref_pic_set_sps_flag, NULL);

  pps = slice_hdr->pps;
  sps = pps->sps;
  rps = &slice_hdr->short_term_ref_pic_sets;

  if (pps->tiles_enabled_flag || pps->entropy_coding_sync_enabled_flag
      || sps->separate_colour_plane_flag || sps->sps_scc_extension_flag
      || pps->pps_scc_extension_flag)
    return NULL;

  for (i = 0; i < rps->NumNegativePics && i < 16; i++)
    num_poc_total_curr += rps->UsedByCurrPicS0[i] != 0;
  for (i = 0; i < rps->NumPositivePics && i < 16; i++)
    num_poc_total_curr += rps->UsedByCurrPicS1[i] != 0;
  if (num_poc_total_curr == 0)
    return NULL;

  /* *INDENT-OFF* */
  hdr = (GstH265SliceHdr) {
    .pps = slice_hdr->pps,
    .first_slice_segment_in_pic_flag = 1,
    .type = GST_H265_P_SLICE,
    .pic_output_flag = 1,
    .pic_order_cnt_lsb = slice_hdr->pic_order_cnt_lsb &
        ((1 << (sps->log2_max_pic_order_cnt_lsb_minus4 + 4)) - 1),
    .short_term_ref_pic_set_sps_flag = 0,
    .short_term_ref_pic_sets = *rps,
    .temporal_mvp_enabled_flag = sps->temporal_mvp_enabled_flag,
    .sao_luma_flag = 0,
    .sao_chroma_flag = 0,
    /* One active reference: the zero merge candidates then all use
     * reference index 0, and collocated_ref_idx is inferred to be 0. */
    .num_ref_idx_active_override_flag = 1,
    .num_ref_idx_l0_active_minus1 = 0,
    .NumPocTotalCurr = num_poc_total_curr,
    .cabac_init_flag =
        pps->cabac_init_present_flag ? slice_hdr->cabac_init_flag : 0,
    .collocated_from_l0_flag = 1,
    .collocated_ref_idx = 0,
    /* A zeroed table writes no explicit weight: default weighting. */
    .pred_weight_table = { 0, },
    .five_minus_max_num_merge_cand = 0,
    .qp_delta = slice_hdr->qp_delta,
    .cb_qp_offset = slice_hdr->cb_qp_offset,
    .cr_qp_offset = slice_hdr->cr_qp_offset,
    /* Nothing to filter; turning it off spares the decoder the pass. */
    .deblocking_filter_override_flag =
        pps->deblocking_filter_override_enabled_flag,
    .deblocking_filter_disabled_flag =
        pps->deblocking_filter_override_enabled_flag ?
        1 : pps->deblocking_filter_disabled_flag,
    .beta_offset_div2 = pps->beta_offset_div2,
    .tc_offset_div2 = pps->tc_offset_div2,
    .loop_filter_across_slices_enabled_flag =
        pps->loop_filter_across_slices_enabled_flag,
    .num_entry_point_offsets = 0,
  };
  /* *INDENT-ON* */

  if (gst_h265_bit_writer_slice_hdr (&hdr, TRUE, GST_H265_NAL_SLICE_TRAIL_N,
          hdr_data, &hdr_size) != GST_H265_BIT_WRITER_OK)
    return NULL;

  /* The slice header ends with byte_alignment (), so the slice data
   * starts on a byte. */
  gst_bit_writer_init (&bw);
  ok = gst_bit_writer_put_bytes (&bw, hdr_data, hdr_size);
  ok &= _write_slice_data (&bw, &hdr);
  if (!ok) {
    gst_bit_writer_reset (&bw);
    return NULL;
  }

  raw_bits = gst_bit_writer_get_size (&bw);
  raw = gst_bit_writer_reset_and_get_data (&bw);

  /* Emulation prevention can add one byte per two input bytes at most. */
  nal_size = raw_bits / 8 + raw_bits / 16 + 8;
  nal = g_malloc (nal_size);
  if (gst_h265_bit_writer_convert_to_nal (4, FALSE, TRUE, FALSE, raw,
          raw_bits, nal, &nal_size) != GST_H265_BIT_WRITER_OK) {
    g_free (raw);
    g_free (nal);
    return NULL;
  }
  g_free (raw);

  return gst_buffer_new_wrapped (nal, nal_size);
}
