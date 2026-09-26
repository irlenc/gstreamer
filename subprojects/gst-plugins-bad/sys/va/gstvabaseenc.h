/* GStreamer
 *  Copyright (C) 2022 Intel Corporation
 *     Author: He Junyan <junyan.he@intel.com>
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
 * License along with this library; if not, write to the0
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#pragma once

#include "gstvadevice.h"
#include "gstvaencoder.h"
#include "gstvaprofile.h"

G_BEGIN_DECLS

#define GST_TYPE_VA_BASE_ENC            (gst_va_base_enc_get_type())
#define GST_VA_BASE_ENC(obj)            (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_VA_BASE_ENC, GstVaBaseEnc))
#define GST_IS_VA_BASE_ENC(obj)         (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_VA_BASE_ENC))
#define GST_VA_BASE_ENC_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST((klass),  GST_TYPE_VA_BASE_ENC, GstVaBaseEncClass))
#define GST_IS_VA_BASE_ENC_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE((klass),  GST_TYPE_VA_BASE_ENC))
#define GST_VA_BASE_ENC_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS((obj),  GST_TYPE_VA_BASE_ENC, GstVaBaseEncClass))

#define GST_VA_BASE_ENC_ENTRYPOINT(obj) (GST_VA_BASE_ENC_GET_CLASS(obj)->entrypoint)

typedef struct _GstVaEncFrame GstVaEncFrame;
typedef struct _GstVaBaseEnc GstVaBaseEnc;
typedef struct _GstVaBaseEncClass GstVaBaseEncClass;
typedef struct _GstVaBaseEncPrivate GstVaBaseEncPrivate;

/* Which recent frames an encoder coded as skip pictures, and the divisor
 * of the input rate its rate control budgets for. */
typedef struct
{
  guint64 history;
  guint history_len;
  guint divisor;
} GstVaCodedRate;

#define GST_TYPE_VA_INTRA_REFRESH_TYPE (gst_va_intra_refresh_type_get_type ())

/* Rolling intra refresh: the P pictures of a cycle each code one stripe
 * of the picture intra, the stripe moving across the picture, so a cycle
 * cleans every region once and a lost picture stops showing after it,
 * without the size spike of an I picture. */
typedef struct
{
  /* VA_ENC_INTRA_REFRESH_ROLLING_COLUMN or _ROW, or 0 when off. */
  guint32 mode;
  /* Refresh units (the driver's) along the direction the stripe moves. */
  guint units;
  /* P pictures that carry a stripe per cycle, and from the start of one
   * cycle to the start of the next. */
  guint cycle_size;
  guint period;
  gint qp_delta;
  /* Where the next P picture stands in the period. */
  guint index;
} GstVaIntraRefresh;

struct _GstVaEncFrame
{
  GstVaEncodePicture *picture;
  /* The input shares its memory with the previous input, so the
   * picture repeats the previous one exactly. */
  gboolean repeat;
};

struct _GstVaBaseEnc
{
  GstVideoEncoder parent_instance;

  GstVaDisplay *display;
  GstVaEncoder *encoder;

  gboolean reconf;
  /* Set when only the rate control targets changed: the subclass may
   * retune the running encoder instead of reopening it, which would
   * start a new IDR. */
  gboolean rc_reconf;

  gboolean is_live;

  VAProfile profile;
  gint width;
  gint height;
  guint rt_format;
  guint codedbuf_size;
  /* The min buffer number required for reorder and output delay. */
  guint min_buffers;

  GstClockTime start_pts;
  GstClockTime frame_duration;

  GQueue reorder_list;
  GQueue ref_list;
  GQueue output_list;
  GstVecDeque *dts_queue;
  guint preferred_output_delay;

  GstVideoCodecState *input_state;
  union {
    GstVideoInfo in_info;
    GstVideoInfoDmaDrm in_drm_info;
  };

  /*< private >*/
  GstVaBaseEncPrivate *priv;

  gpointer _padding[GST_PADDING];
};

struct _GstVaBaseEncClass
{
  GstVideoEncoderClass parent_class;

  void     (*reset_state)    (GstVaBaseEnc * encoder);
  gboolean (*reconfig)       (GstVaBaseEnc * encoder);
  /* Optional. Returns FALSE when the change needs a full reconfig. */
  gboolean (*reconfig_rate_control) (GstVaBaseEnc * encoder);
  gboolean (*new_frame)      (GstVaBaseEnc * encoder,
                              GstVideoCodecFrame * frame);
  gboolean (*reorder_frame)  (GstVaBaseEnc * base,
                              GstVideoCodecFrame * frame,
                              gboolean bump_all,
                              GstVideoCodecFrame ** out_frame);
  GstFlowReturn (*encode_frame) (GstVaBaseEnc * encoder,
                                 GstVideoCodecFrame * frame,
                                 gboolean is_last);
  gboolean (*prepare_output) (GstVaBaseEnc * encoder,
                              GstVideoCodecFrame * frame,
                              gboolean * complete);

  GstVaCodecs codec;
  VAEntrypoint entrypoint;
  gchar *render_device_path;

  gpointer _padding[GST_PADDING];
};

struct CData
{
  VAEntrypoint entrypoint;
  gchar *render_device_path;
  gchar *description;
  GstCaps *sink_caps;
  GstCaps *src_caps;
};

GType                 gst_va_base_enc_get_type            (void);

gboolean              gst_va_base_enc_add_rate_control_parameter (GstVaBaseEnc * base,
                                                                  GstVaEncodePicture * picture,
                                                                  guint32 rc_mode,
                                                                  guint max_bitrate_bits,
                                                                  guint target_percentage,
                                                                  guint32 qp_i,
                                                                  guint32 min_qp,
                                                                  guint32 max_qp,
                                                                  guint32 mbbrc);
gboolean              gst_va_base_enc_add_quality_level_parameter (GstVaBaseEnc * base,
                                                                   GstVaEncodePicture * picture,
                                                                   guint target_usage);
gboolean              gst_va_base_enc_add_frame_rate_parameter (GstVaBaseEnc * base,
                                                                GstVaEncodePicture * picture);
void                  gst_va_coded_rate_reset             (GstVaCodedRate * rate);
gboolean              gst_va_coded_rate_push              (GstVaCodedRate * rate,
                                                           gboolean skipped);
gboolean              gst_va_base_enc_add_frame_rate_parameter_divided (GstVaBaseEnc * base,
                                                                        GstVaEncodePicture * picture,
                                                                        guint divisor);
gboolean              gst_va_base_enc_add_hrd_parameter   (GstVaBaseEnc * base,
                                                           GstVaEncodePicture * picture,
                                                           guint32 rc_mode,
                                                           guint cpb_length_bits);
gboolean              gst_va_base_enc_add_trellis_parameter (GstVaBaseEnc * base,
                                                             GstVaEncodePicture * picture,
                                                             gboolean use_trellis);
gboolean              gst_va_base_enc_add_intra_refresh_parameter (GstVaBaseEnc * base,
                                                                   GstVaEncodePicture * picture,
                                                                   guint32 mode,
                                                                   guint16 location,
                                                                   guint16 size,
                                                                   gint8 qp_delta);
GType                 gst_va_intra_refresh_type_get_type  (void);
gboolean              gst_va_intra_refresh_setup          (GstVaIntraRefresh * ir,
                                                           GstVaBaseEnc * base,
                                                           guint32 mode,
                                                           guint cycle_size,
                                                           guint cycle_dist,
                                                           gint qp_delta,
                                                           guint width_units,
                                                           guint height_units);
void                  gst_va_intra_refresh_check_gop      (GstVaIntraRefresh * ir,
                                                           GstVaBaseEnc * base,
                                                           guint num_bframes,
                                                           guint num_refs);
gboolean              gst_va_intra_refresh_add            (GstVaIntraRefresh * ir,
                                                           GstVaBaseEnc * base,
                                                           GstVaEncodePicture * picture);
void                  gst_va_intra_refresh_restart        (GstVaIntraRefresh * ir);
gboolean              gst_va_intra_refresh_cycle_starts   (const GstVaIntraRefresh * ir);
guint                 gst_va_base_enc_min_cpb_size        (GstVaBaseEnc * base,
                                                           guint max_bitrate,
                                                           gboolean intra_refresh);
void                  gst_va_base_enc_add_codec_tag       (GstVaBaseEnc * base,
                                                           const gchar * codec_name);
void                  gst_va_base_enc_reset_state         (GstVaBaseEnc * base);
guint                 gst_va_base_enc_adjust_coded_buffer_size (GstVaBaseEnc * base,
                                                                 guint codedbuf_size,
                                                                 guint32 rc_ctrl_mode,
                                                                 guint bitrate_bits);
GstBuffer *           gst_va_base_enc_create_output_buffer (GstVaBaseEnc * base,
                                                            GstVaEncodePicture * picture,
                                                            const guint8 * prefix_data,
                                                            guint prefix_data_len);
gint                  gst_va_base_enc_copy_output_data    (GstVaBaseEnc * base,
                                                           GstVaEncodePicture * picture,
                                                           guint8 * data,
                                                           gint size);
void                  gst_va_base_enc_push_dts            (GstVaBaseEnc * base,
                                                           GstVideoCodecFrame * frame,
                                                           guint max_reorder_num);
GstClockTime          gst_va_base_enc_pop_dts             (GstVaBaseEnc * base);
void                  gst_va_base_enc_update_property_uint (GstVaBaseEnc * base,
                                                            guint32 * old_val,
                                                            guint32 new_val,
                                                            GParamSpec * pspec);
void                  gst_va_base_enc_update_property_bool (GstVaBaseEnc * base,
                                                            gboolean * old_val,
                                                            gboolean new_val,
                                                            GParamSpec * pspec);

static inline gpointer
gst_va_get_enc_frame (GstVideoCodecFrame * frame)
{
  GstVaEncFrame *enc_frame = gst_video_codec_frame_get_user_data (frame);
  g_assert (enc_frame);

  return enc_frame;
}

static inline void
gst_va_set_enc_frame (GstVideoCodecFrame * frame,
    GstVaEncFrame * frame_in, GDestroyNotify notify)
{
  gst_video_codec_frame_set_user_data (frame, frame_in, notify);
}

G_DEFINE_AUTOPTR_CLEANUP_FUNC(GstVaBaseEnc, gst_object_unref)

G_END_DECLS
