/*
 * VC-1 and WMV3 decoder
 * Copyright (c) 2011 Mashiat Sarker Shakkhar
 * Copyright (c) 2006-2007 Konstantin Shishkov
 * Partly based on vc9.c (c) 2005 Anonymous, Alex Beregszaszi, Michael Niedermayer
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * VC-1 and WMV3 block decoding routines
 */

#include "avcodec.h"
#include "mpegutils.h"
#include "mpegvideo.h"
#include "msmpeg4data.h"
#include "unary.h"
#include "vc1.h"
#include "vc1_pred.h"
#include "vc1acdata.h"
#include "vc1data.h"

#define MB_INTRA_VLC_BITS 9
#define DC_VLC_BITS 9

// offset tables for interlaced picture MVDATA decoding
static const uint8_t offset_table[2][9] = {
    {  0,  1,  2,  4,  8, 16, 32,  64, 128 },
    {  0,  1,  3,  7, 15, 31, 63, 127, 255 },
};

// mapping table for internal block representation
static const int block_map[6] = {0, 2, 1, 3, 4, 5};

/***********************************************************************/
/**
 * @name VC-1 Bitplane decoding
 * @see 8.7, p56
 * @{
 */


static inline void init_block_index(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    ff_init_block_index(s);
    if (v->field_mode && !(v->second_field ^ v->tff)) {
        s->dest[0] += s->current_picture_ptr->f->linesize[0];
        s->dest[1] += s->current_picture_ptr->f->linesize[1];
        s->dest[2] += s->current_picture_ptr->f->linesize[2];
    }
}

static inline void update_block_index(VC1MBCtx *mbctx,
                                      int blkidx[BLOCKIDX_MAX],
                                      int mb_width,
                                      int num_mb,
                                      int i)
{
    static const int blkidx_init[BLOCKIDX_MAX] = {  0,  0,
                                                   -1,
                                                   -1, -1, -1, -1,
                                                   -1, -1, -1,
                                                   -1, -1, -1, -1  };
    int curr_mbidx = i % (2 * mb_width + 3);
    int left_mbidx = curr_mbidx > 0 ? curr_mbidx - 1 : 2 * mb_width + 2;
    int curr_blkidx = i % (mb_width + 2);
    int mb_x = i % mb_width;
    int mquant_selector = MQUANT_SELECT_PQUANT;

    if (i == 0) { /* First block */
        memcpy(blkidx, blkidx_init, sizeof(blkidx_init));
        mbctx->mquant_selector = mbctx->dqedge & DQEDGE_LEFTTOP ? MQUANT_SELECT_ALTPQUANT : MQUANT_SELECT_PQUANT;

        return;
    }

    blkidx[MBIDX] = curr_mbidx;

    blkidx[BLOCKIDX_L1] = blkidx[BLOCKIDX_Y0] + 2;
    blkidx[BLOCKIDX_L3] = blkidx[BLOCKIDX_Y0] + 3;
    blkidx[BLOCKIDX_CB_L] = blkidx[BLOCKIDX_Y0] + 4;
    blkidx[BLOCKIDX_CR_L] = blkidx[BLOCKIDX_Y0] + 5;

    blkidx[BLOCKIDX_LT3] = blkidx[BLOCKIDX_T3];
    blkidx[BLOCKIDX_CB_LT] = blkidx[BLOCKIDX_T3] + 1;
    blkidx[BLOCKIDX_CR_LT] = blkidx[BLOCKIDX_T3] + 2;

    blkidx[BLOCKIDX_Y0] = curr_blkidx > 0 ? blkidx[BLOCKIDX_Y0] + 6 : 0;

    blkidx[BLOCKIDX_T2] = curr_blkidx == mb_width ? 1 : blkidx[BLOCKIDX_T2] + 6;
    blkidx[BLOCKIDX_T3] = blkidx[BLOCKIDX_T2] + 2;
    blkidx[BLOCKIDX_CB_T] = blkidx[BLOCKIDX_T2] + 3;
    blkidx[BLOCKIDX_CR_T] = blkidx[BLOCKIDX_T2] + 4;

    blkidx[BLOCKIDX_RT2] = curr_blkidx == mb_width - 1 ? 1 : blkidx[BLOCKIDX_T2] + 6;

    mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_LUMA] = mbctx->s_mbctx[left_mbidx].dest[COMPONENT_LUMA] + 16;
    mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_CB] = mbctx->s_mbctx[left_mbidx].dest[COMPONENT_CB] + 8;
    mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_CR] = mbctx->s_mbctx[left_mbidx].dest[COMPONENT_CR] + 8;

    if (i < mb_width) { /* Top row */
        memcpy(blkidx + BLOCKIDX_RT2, blkidx_init + BLOCKIDX_RT2, sizeof(blkidx_init[0]) * 8);
        mquant_selector |= mbctx->dqedge & DQEDGE_TOP ? MQUANT_SELECT_ALTPQUANT : 0;
/*
        blkidx[BLOCKIDX_RT2] = -1;
        blkidx[BLOCKIDX_T2] = -1;
        blkidx[BLOCKIDX_T3] = -1;
        blkidx[BLOCKIDX_CB_T] = -1;
        blkidx[BLOCKIDX_CR_T] = -1;
        blkidx[BLOCKIDX_LT3] = -1;
        blkidx[BLOCKIDX_CB_LT] = -1;
        blkidx[BLOCKIDX_CR_LT] = -1;
*/
    }

    if (mb_x == 0) { /* Leftmost block */
        int top_mbidx = curr_mbidx < mb_width ? curr_mbidx + mb_width + 3 : curr_mbidx - mb_width;

        memcpy(blkidx + BLOCKIDX_LT3, blkidx_init + BLOCKIDX_LT3, sizeof(blkidx_init[0]) * 7);
        mquant_selector |= mbctx->dqedge & DQEDGE_LEFT ? MQUANT_SELECT_ALTPQUANT : 0;
/*
        blkidx[BLOCKIDX_LT3] = -1;
        blkidx[BLOCKIDX_CB_LT] = -1;
        blkidx[BLOCKIDX_CR_LT] = -1;
        blkidx[BLOCKIDX_L1] = -1;
        blkidx[BLOCKIDX_L3] = -1;
        blkidx[BLOCKIDX_CB_L] = -1;
        blkidx[BLOCKIDX_CR_L] = -1;
*/
        mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_LUMA] = mbctx->s_mbctx[top_mbidx].dest[COMPONENT_LUMA] + 16 * mbctx->linesize[COMPONENT_TYPE_LUMA];
        mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_CB] = mbctx->s_mbctx[top_mbidx].dest[COMPONENT_CB] + 8 * mbctx->linesize[COMPONENT_TYPE_CHROMA];
        mbctx->s_mbctx[curr_mbidx].dest[COMPONENT_CR] = mbctx->s_mbctx[top_mbidx].dest[COMPONENT_CR] + 8 * mbctx->linesize[COMPONENT_TYPE_CHROMA];
    }

    if (mb_x == mb_width - 1) { /* Rightmost block */
        blkidx[BLOCKIDX_RT2] = -1;
        mquant_selector |= mbctx->dqedge & DQEDGE_RIGHT ? MQUANT_SELECT_ALTPQUANT : 0;
    }

    if (i >= num_mb - mb_width) /* Bottom row */
        mquant_selector |= mbctx->dqedge & DQEDGE_BOTTOM ? MQUANT_SELECT_ALTPQUANT : 0;

    mbctx->mquant_selector = mquant_selector;
}

/** @} */ //Bitplane group

static void vc1_put_blocks_clamped(VC1Context *v, int put_signed)
{
    MpegEncContext *s = &v->s;
    uint8_t *dest;
    int block_count = CONFIG_GRAY && (s->avctx->flags & AV_CODEC_FLAG_GRAY) ? 4 : 6;
    int fieldtx = 0;
    int i;

    /* The put pixels loop is one MB row and one MB column behind the decoding
     * loop because we can only put pixels when overlap filtering is done. For
     * interlaced frame pictures, however, the put pixels loop is only one
     * column behind the decoding loop as interlaced frame pictures only need
     * horizontal overlap filtering. */
    if (!s->first_slice_line && v->fcm != ILACE_FRAME) {
        if (s->mb_x) {
            for (i = 0; i < block_count; i++) {
                if (i > 3 ? v->mb_type[0][s->block_index[i] - s->block_wrap[i] - 1] :
                            v->mb_type[0][s->block_index[i] - 2 * s->block_wrap[i] - 2]) {
                    dest = s->dest[0] + ((i & 2) - 4) * 4 * s->linesize + ((i & 1) - 2) * 8;
                    if (put_signed)
                        s->idsp.put_signed_pixels_clamped(v->block[v->topleft_blk_idx][block_map[i]],
                                                          i > 3 ? s->dest[i - 3] - 8 * s->uvlinesize - 8 : dest,
                                                          i > 3 ? s->uvlinesize : s->linesize);
                    else
                        s->idsp.put_pixels_clamped(v->block[v->topleft_blk_idx][block_map[i]],
                                                   i > 3 ? s->dest[i - 3] - 8 * s->uvlinesize - 8 : dest,
                                                   i > 3 ? s->uvlinesize : s->linesize);
                }
            }
        }
        if (s->mb_x == v->end_mb_x - 1) {
            for (i = 0; i < block_count; i++) {
                if (i > 3 ? v->mb_type[0][s->block_index[i] - s->block_wrap[i]] :
                            v->mb_type[0][s->block_index[i] - 2 * s->block_wrap[i]]) {
                    dest = s->dest[0] + ((i & 2) - 4) * 4 * s->linesize + (i & 1) * 8;
                    if (put_signed)
                        s->idsp.put_signed_pixels_clamped(v->block[v->top_blk_idx][block_map[i]],
                                                          i > 3 ? s->dest[i - 3] - 8 * s->uvlinesize : dest,
                                                          i > 3 ? s->uvlinesize : s->linesize);
                    else
                        s->idsp.put_pixels_clamped(v->block[v->top_blk_idx][block_map[i]],
                                                   i > 3 ? s->dest[i - 3] - 8 * s->uvlinesize : dest,
                                                   i > 3 ? s->uvlinesize : s->linesize);
                }
            }
        }
    }
    if (s->mb_y == s->end_mb_y - 1 || v->fcm == ILACE_FRAME) {
        if (s->mb_x) {
            if (v->fcm == ILACE_FRAME)
                fieldtx = v->fieldtx_plane[s->mb_y * s->mb_stride + s->mb_x - 1];
            for (i = 0; i < block_count; i++) {
                if (i > 3 ? v->mb_type[0][s->block_index[i] - 1] :
                            v->mb_type[0][s->block_index[i] - 2]) {
                    if (fieldtx)
                        dest = s->dest[0] + ((i & 2) >> 1) * s->linesize + ((i & 1) - 2) * 8;
                    else
                        dest = s->dest[0] + (i & 2) * 4 * s->linesize + ((i & 1) - 2) * 8;
                    if (put_signed)
                        s->idsp.put_signed_pixels_clamped(v->block[v->left_blk_idx][block_map[i]],
                                                          i > 3 ? s->dest[i - 3] - 8 : dest,
                                                          i > 3 ? s->uvlinesize : s->linesize << fieldtx);
                    else
                        s->idsp.put_pixels_clamped(v->block[v->left_blk_idx][block_map[i]],
                                                   i > 3 ? s->dest[i - 3] - 8 : dest,
                                                   i > 3 ? s->uvlinesize : s->linesize << fieldtx);
                }
            }
        }
        if (s->mb_x == v->end_mb_x - 1) {
            if (v->fcm == ILACE_FRAME)
                fieldtx = v->fieldtx_plane[s->mb_y * s->mb_stride + s->mb_x];
            for (i = 0; i < block_count; i++) {
                if (v->mb_type[0][s->block_index[i]]) {
                    if (fieldtx)
                        dest = s->dest[0] + ((i & 2) >> 1) * s->linesize + (i & 1) * 8;
                    else
                        dest = s->dest[0] + (i & 2) * 4 * s->linesize + (i & 1) * 8;
                    if (put_signed)
                        s->idsp.put_signed_pixels_clamped(v->block[v->cur_blk_idx][block_map[i]],
                                                          i > 3 ? s->dest[i - 3] : dest,
                                                          i > 3 ? s->uvlinesize : s->linesize << fieldtx);
                    else
                        s->idsp.put_pixels_clamped(v->block[v->cur_blk_idx][block_map[i]],
                                                   i > 3 ? s->dest[i - 3] : dest,
                                                   i > 3 ? s->uvlinesize : s->linesize << fieldtx);
                }
            }
        }
    }
}

static void vc1_put_pixels(VC1MBCtx *mbctx, int mb_blkidx, uint8_t **dest) {
    VC1StoredBlkCtx *sblkctx = mbctx->s_blkctx;
    int16_t (*block)[64] = mbctx->block + mb_blkidx;
    ptrdiff_t linesize;

    linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];

    /* block Y0 */
    if (sblkctx[mb_blkidx].btype == BLOCK_INTRA)
        mbctx->put_pixels(block[0],
                          dest[COMPONENT_LUMA],
                          linesize);

    /* block Y2 */
    if (sblkctx[mb_blkidx + 1].btype == BLOCK_INTRA)
        mbctx->put_pixels(block[1],
                          dest[COMPONENT_LUMA] + 8 * linesize,
                          linesize);

    /* block Y1 */
    if (sblkctx[mb_blkidx + 2].btype == BLOCK_INTRA)
        mbctx->put_pixels(block[2],
                          dest[COMPONENT_LUMA] + 8,
                          linesize);

    /* block Y3 */
    if (sblkctx[mb_blkidx + 3].btype == BLOCK_INTRA)
        mbctx->put_pixels(block[3],
                          dest[COMPONENT_LUMA] + 8 * linesize + 8,
                          linesize);

    linesize = mbctx->linesize[COMPONENT_TYPE_CHROMA];

    /* block Cb */
    if (sblkctx[mb_blkidx + 4].btype == BLOCK_INTRA) {
//    if (sblkctx[mb_blkidx + 5].btype == BLOCK_INTRA)
        mbctx->put_pixels(block[4],
                          dest[COMPONENT_CB],
                          linesize);

    /* block Cr */
        mbctx->put_pixels(block[5],
                          dest[COMPONENT_CR],
                          linesize);
    }
}

#define inc_blk_idx(idx) do { \
        idx++; \
        if (idx >= v->n_allocated_blks) \
            idx = 0; \
    } while (0)

/***********************************************************************/
/**
 * @name VC-1 Block-level functions
 * @see 7.1.4, p91 and 8.1.1.7, p(1)04
 * @{
 */

/**
 * @def GET_MQUANT
 * @brief Get macroblock-level quantizer scale
 */
#define GET_MQUANT()                                           \
    if (v->dquantfrm) {                                        \
        int edges = 0;                                         \
        if (v->dqprofile == DQPROFILE_ALL_MBS) {               \
            if (v->dqbilevel) {                                \
                mquant = (get_bits1(gb)) ? -v->altpq : v->pq;  \
            } else {                                           \
                mqdiff = get_bits(gb, 3);                      \
                if (mqdiff != 7)                               \
                    mquant = -v->pq - mqdiff;                  \
                else                                           \
                    mquant = -get_bits(gb, 5);                 \
            }                                                  \
        }                                                      \
        if (v->dqprofile == DQPROFILE_SINGLE_EDGE)             \
            edges = 1 << v->dqsbedge;                          \
        else if (v->dqprofile == DQPROFILE_DOUBLE_EDGE)        \
            edges = (3 << v->dqsbedge) % 15;                   \
        else if (v->dqprofile == DQPROFILE_FOUR_EDGES)         \
            edges = 15;                                        \
        if ((edges&1) && !s->mb_x)                             \
            mquant = -v->altpq;                                \
        if ((edges&2) && !s->mb_y)                             \
            mquant = -v->altpq;                                \
        if ((edges&4) && s->mb_x == (s->mb_width - 1))         \
            mquant = -v->altpq;                                \
        if ((edges&8) &&                                       \
            s->mb_y == ((s->mb_height >> v->field_mode) - 1))  \
            mquant = -v->altpq;                                \
        if (!mquant || mquant > 31 || mquant < -31) {          \
            av_log(v->s.avctx, AV_LOG_ERROR,                   \
                   "Overriding invalid mquant %d\n", mquant);  \
            mquant = 1;                                        \
        }                                                      \
    }

static void vc1_decode_mquant(VC1PMBCtx *mbctx, GetBitContext *gb)
{
    int dqbilevel = mbctx->dqedge & DQEDGE_DQBILEVEL;
    int mqdiff;

    if (mbctx->dqedge & DQEDGE_ALL_MBS) {
        if (dqbilevel) {
            mqdiff = get_bits1(gb); // MQDIFF
            mbctx->mquant_selector = mqdiff ? MQUANT_SELECT_ALTPQUANT : MQUANT_SELECT_PQUANT;
        } else {
            mqdiff = get_bits(gb, 3); // MQDIFF
            mbctx->mquant_selector = MQUANT_SELECT_ALTPQUANT;
            mbctx->mquant = av_clip(mqdiff == 7 ? mbctx->pquant + mqdiff : get_bits(gb, 5), 1, 31);

            return;
        }
    }

    mbctx->mquant = mbctx->mquant_selector == MQUANT_SELECT_ALTPQUANT ?
                    mbctx->altpquant : mbctx->pquant;
}

/**
 * @def GET_MVDATA(_dmv_x, _dmv_y)
 * @brief Get MV differentials
 * @see MVDATA decoding from 8.3.5.2, p(1)20
 * @param _dmv_x Horizontal differential for decoded MV
 * @param _dmv_y Vertical differential for decoded MV
 */
#define GET_MVDATA(_dmv_x, _dmv_y)                                      \
    index = 1 + get_vlc2(gb, ff_vc1_mv_diff_vlc[s->mv_table_index].table, \
                         VC1_MV_DIFF_VLC_BITS, 2);                      \
    if (index > 36) {                                                   \
        mb_has_coeffs = 1;                                              \
        index -= 37;                                                    \
    } else                                                              \
        mb_has_coeffs = 0;                                              \
    s->mb_intra = 0;                                                    \
    if (!index) {                                                       \
        _dmv_x = _dmv_y = 0;                                            \
    } else if (index == 35) {                                           \
        _dmv_x = get_bits(gb, v->k_x - 1 + s->quarter_sample);          \
        _dmv_y = get_bits(gb, v->k_y - 1 + s->quarter_sample);          \
    } else if (index == 36) {                                           \
        _dmv_x = 0;                                                     \
        _dmv_y = 0;                                                     \
        s->mb_intra = 1;                                                \
    } else {                                                            \
        index1 = index % 6;                                             \
        _dmv_x = offset_table[1][index1];                               \
        val = size_table[index1] - (!s->quarter_sample && index1 == 5); \
        if (val > 0) {                                                  \
            val = get_bits(gb, val);                                    \
            sign = 0 - (val & 1);                                       \
            _dmv_x = (sign ^ ((val >> 1) + _dmv_x)) - sign;             \
        }                                                               \
                                                                        \
        index1 = index / 6;                                             \
        _dmv_y = offset_table[1][index1];                               \
        val = size_table[index1] - (!s->quarter_sample && index1 == 5); \
        if (val > 0) {                                                  \
            val = get_bits(gb, val);                                    \
            sign = 0 - (val & 1);                                       \
            _dmv_y = (sign ^ ((val >> 1) + _dmv_y)) - sign;             \
        }                                                               \
    }

static void vc1_decode_mv_diff(VC1PMBCtx *mbctx,
                               int curr_blkidx,
                               GetBitContext *gb)
{
    static const uint8_t size_table[6] = { 0, 2, 3, 4, 5 ,8 };
    static const uint8_t offset_table[6] = { 0, 1, 3, 7, 15, 31 };
    static const uint8_t k_x[4] = { 9, 10, 12, 13 };

    VC1StoredBlkCtx *curr_sblkctx = mbctx->s_blkctx + curr_blkidx;
    int halfpel_flag = mbctx->mvmode & 1 ? 0 : 1;
    int index, index1, val, sign;

    curr_sblkctx->btype = BLOCK_INTER;
    curr_sblkctx->mv[MV_CTX_FORWARD][MV_X] = 0;
    curr_sblkctx->mv[MV_CTX_FORWARD][MV_Y] = 0;

    if (!curr_sblkctx->is_coded)
        return;

    index = get_vlc2(gb, mbctx->mv_diff_vlc->table, 8, 2) + 1; // MVDATA
    curr_sblkctx->is_coded = index >= 37;
    index = index >= 37 ? index - 37 : index;

    switch (index) {
    case 0:
        break;

    case 36:
        curr_sblkctx->btype = BLOCK_INTRA;
        break;

    case 35:
        curr_sblkctx->mv[MV_CTX_FORWARD][MV_X] = get_bits(gb, k_x[mbctx->mvrange] - halfpel_flag);
        curr_sblkctx->mv[MV_CTX_FORWARD][MV_Y] = get_bits(gb, mbctx->mvrange + 8 - halfpel_flag);
        break;

    default:
        index1 = index % 6;
        if (size_table[index1]) {
            val = get_bits(gb, size_table[index1] - (index1 == 5 ? halfpel_flag : 0));
            sign = val & 1;
            curr_sblkctx->mv[MV_CTX_FORWARD][MV_X] = (((val >> 1) + offset_table[index1]) ^ -sign) + sign;
        }

        index1 = index / 6;
        if (size_table[index1]) {
            val = get_bits(gb, size_table[index1] - (index1 == 5 ? halfpel_flag : 0));
            sign = val & 1;
            curr_sblkctx->mv[MV_CTX_FORWARD][MV_Y] = (((val >> 1) + offset_table[index1]) ^ -sign) + sign;
        }
        break;
    }
}

static av_always_inline void get_mvdata_interlaced(VC1Context *v, int *dmv_x,
                                                   int *dmv_y, int *pred_flag)
{
    int index, index1;
    int extend_x, extend_y;
    GetBitContext *gb = &v->s.gb;
    int bits, esc;
    int val, sign;

    if (v->numref) {
        bits = VC1_2REF_MVDATA_VLC_BITS;
        esc  = 125;
    } else {
        bits = VC1_1REF_MVDATA_VLC_BITS;
        esc  = 71;
    }
    extend_x = v->dmvrange & 1;
    extend_y = (v->dmvrange >> 1) & 1;
    index = get_vlc2(gb, v->imv_vlc->table, bits, 3);
    if (index == esc) {
        *dmv_x = get_bits(gb, v->k_x);
        *dmv_y = get_bits(gb, v->k_y);
        if (v->numref) {
            if (pred_flag)
                *pred_flag = *dmv_y & 1;
            *dmv_y = (*dmv_y + (*dmv_y & 1)) >> 1;
        }
    }
    else {
        av_assert0(index < esc);
        index1 = (index + 1) % 9;
        if (index1 != 0) {
            val    = get_bits(gb, index1 + extend_x);
            sign   = 0 - (val & 1);
            *dmv_x = (sign ^ ((val >> 1) + offset_table[extend_x][index1])) - sign;
        } else
            *dmv_x = 0;
        index1 = (index + 1) / 9;
        if (index1 > v->numref) {
            val    = get_bits(gb, (index1 >> v->numref) + extend_y);
            sign   = 0 - (val & 1);
            *dmv_y = (sign ^ ((val >> 1) + offset_table[extend_y][index1 >> v->numref])) - sign;
        } else
            *dmv_y = 0;
        if (v->numref && pred_flag)
            *pred_flag = index1 & 1;
    }
}

/** Reconstruct motion vector for B-frame and do motion compensation
 */
static inline void vc1_b_mc(VC1Context *v, int dmv_x[2], int dmv_y[2],
                            int direct, int mode)
{
    if (direct) {
        ff_vc1_mc_1mv(v, 0);
        ff_vc1_interp_mc(v);
        return;
    }
    if (mode == BMV_TYPE_INTERPOLATED) {
        ff_vc1_mc_1mv(v, 0);
        ff_vc1_interp_mc(v);
        return;
    }

    ff_vc1_mc_1mv(v, (mode == BMV_TYPE_BACKWARD));
}

/** Get predicted DC value for I-frames only
 * prediction dir: left=0, top=1
 * @param s MpegEncContext
 * @param overlap flag indicating that overlap filtering is used
 * @param pq integer part of picture quantizer
 * @param[in] n block index in the current MB
 * @param dc_val_ptr Pointer to DC predictor
 * @param dir_ptr Prediction direction for use in AC prediction
 */
static inline int vc1_i_pred_dc(MpegEncContext *s, int overlap, int pq, int n,
                                int16_t **dc_val_ptr, int *dir_ptr)
{
    int a, b, c, wrap, pred, scale;
    int16_t *dc_val;
    static const uint16_t dcpred[32] = {
        -1, 1024,  512,  341,  256,  205,  171,  146,  128,
             114,  102,   93,   85,   79,   73,   68,   64,
              60,   57,   54,   51,   49,   47,   45,   43,
              41,   39,   38,   37,   35,   34,   33
    };

    /* find prediction - wmv3_dc_scale always used here in fact */
    if (n < 4) scale = s->y_dc_scale;
    else       scale = s->c_dc_scale;

    wrap   = s->block_wrap[n];
    dc_val = s->dc_val[0] + s->block_index[n];

    /* B A
     * C X
     */
    c = dc_val[ - 1];
    b = dc_val[ - 1 - wrap];
    a = dc_val[ - wrap];

    if (pq < 9 || !overlap) {
        /* Set outer values */
        if (s->first_slice_line && (n != 2 && n != 3))
            b = a = dcpred[scale];
        if (s->mb_x == 0 && (n != 1 && n != 3))
            b = c = dcpred[scale];
    } else {
        /* Set outer values */
        if (s->first_slice_line && (n != 2 && n != 3))
            b = a = 0;
        if (s->mb_x == 0 && (n != 1 && n != 3))
            b = c = 0;
    }

    if (abs(a - b) <= abs(b - c)) {
        pred     = c;
        *dir_ptr = 1; // left
    } else {
        pred     = a;
        *dir_ptr = 0; // top
    }

    /* update predictor */
    *dc_val_ptr = &dc_val[0];
    return pred;
}

/** Get predicted DC value
 * prediction dir: left=0, top=1
 * @param s MpegEncContext
 * @param overlap flag indicating that overlap filtering is used
 * @param pq integer part of picture quantizer
 * @param[in] n block index in the current MB
 * @param a_avail flag indicating top block availability
 * @param c_avail flag indicating left block availability
 * @param dc_val_ptr Pointer to DC predictor
 * @param dir_ptr Prediction direction for use in AC prediction
 */
static inline int ff_vc1_pred_dc(MpegEncContext *s, int overlap, int pq, int n,
                              int a_avail, int c_avail,
                              int16_t **dc_val_ptr, int *dir_ptr)
{
    int a, b, c, wrap, pred;
    int16_t *dc_val;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int q1, q2 = 0;
    int dqscale_index;

    /* scale predictors if needed */
    q1 = FFABS(s->current_picture.qscale_table[mb_pos]);
    dqscale_index = s->y_dc_scale_table[q1];
    if (dqscale_index < 0)
        return 0;

    wrap = s->block_wrap[n];
    dc_val = s->dc_val[0] + s->block_index[n];

    /* B A
     * C X
     */
    c = dc_val[ - 1];
    b = dc_val[ - 1 - wrap];
    a = dc_val[ - wrap];

    if (c_avail && (n != 1 && n != 3)) {
        q2 = FFABS(s->current_picture.qscale_table[mb_pos - 1]);
        if (q2 && q2 != q1)
            c = (int)((unsigned)c * s->y_dc_scale_table[q2] * ff_vc1_dqscale[dqscale_index] + 0x20000) >> 18;
    }
    if (a_avail && (n != 2 && n != 3)) {
        q2 = FFABS(s->current_picture.qscale_table[mb_pos - s->mb_stride]);
        if (q2 && q2 != q1)
            a = (int)((unsigned)a * s->y_dc_scale_table[q2] * ff_vc1_dqscale[dqscale_index] + 0x20000) >> 18;
    }
    if (a_avail && c_avail && (n != 3)) {
        int off = mb_pos;
        if (n != 1)
            off--;
        if (n != 2)
            off -= s->mb_stride;
        q2 = FFABS(s->current_picture.qscale_table[off]);
        if (q2 && q2 != q1)
            b = (int)((unsigned)b * s->y_dc_scale_table[q2] * ff_vc1_dqscale[dqscale_index] + 0x20000) >> 18;
    }

    if (c_avail && (!a_avail || abs(a - b) <= abs(b - c))) {
        pred     = c;
        *dir_ptr = 1; // left
    } else if (a_avail) {
        pred     = a;
        *dir_ptr = 0; // top
    } else {
        pred     = 0;
        *dir_ptr = 1; // left
    }

    /* update predictor */
    *dc_val_ptr = &dc_val[0];
    return pred;
}

/** @} */ // Block group

/**
 * @name VC1 Macroblock-level functions in Simple/Main Profiles
 * @see 7.1.4, p91 and 8.1.1.7, p(1)04
 * @{
 */

static inline int vc1_coded_block_pred(MpegEncContext * s, int n,
                                       uint8_t **coded_block_ptr)
{
    int xy, wrap, pred, a, b, c;

    xy   = s->block_index[n];
    wrap = s->b8_stride;

    /* B C
     * A X
     */
    a = s->coded_block[xy - 1       ];
    b = s->coded_block[xy - 1 - wrap];
    c = s->coded_block[xy     - wrap];

    if (b == c) {
        pred = a;
    } else {
        pred = c;
    }

    /* store value */
    *coded_block_ptr = &s->coded_block[xy];

    return pred;
}

/**
 * Decode one AC coefficient
 * @param v The VC1 context
 * @param last Last coefficient
 * @param skip How much zero coefficients to skip
 * @param value Decoded AC coefficient value
 * @param codingset set of VLC to decode data
 * @see 8.1.3.4
 */
static int vc1_decode_ac_coeff(VC1Context *v, int *last, int *skip,
                                int *value, int codingset)
{
    GetBitContext *gb = &v->s.gb;
    int index, run, level, lst, sign;

    index = get_vlc2(gb, ff_vc1_ac_coeff_table[codingset].table, AC_VLC_BITS, 3);
    if (index < 0)
        return index;
    if (index != ff_vc1_ac_sizes[codingset] - 1) {
        run   = vc1_index_decode_table[codingset][index][0];
        level = vc1_index_decode_table[codingset][index][1];
        lst   = index >= vc1_last_decode_table[codingset] ||
                get_bits_left(gb) < 0;
        sign  = get_bits1(gb);
    } else {
        int escape = decode210(gb);
        if (escape != 2) {
            index = get_vlc2(gb, ff_vc1_ac_coeff_table[codingset].table, AC_VLC_BITS, 3);
            if (index >= ff_vc1_ac_sizes[codingset] - 1U)
                return AVERROR_INVALIDDATA;
            run   = vc1_index_decode_table[codingset][index][0];
            level = vc1_index_decode_table[codingset][index][1];
            lst   = index >= vc1_last_decode_table[codingset];
            if (escape == 0) {
                if (lst)
                    level += vc1_last_delta_level_table[codingset][run];
                else
                    level += vc1_delta_level_table[codingset][run];
            } else {
                if (lst)
                    run += vc1_last_delta_run_table[codingset][level] + 1;
                else
                    run += vc1_delta_run_table[codingset][level] + 1;
            }
            sign = get_bits1(gb);
        } else {
            lst = get_bits1(gb);
            if (v->s.esc3_level_length == 0) {
                if (v->pq < 8 || v->dquantfrm) { // table 59
                    v->s.esc3_level_length = get_bits(gb, 3);
                    if (!v->s.esc3_level_length)
                        v->s.esc3_level_length = get_bits(gb, 2) + 8;
                } else { // table 60
                    v->s.esc3_level_length = get_unary(gb, 1, 6) + 2;
                }
                v->s.esc3_run_length = 3 + get_bits(gb, 2);
            }
            run   = get_bits(gb, v->s.esc3_run_length);
            sign  = get_bits1(gb);
            level = get_bits(gb, v->s.esc3_level_length);
        }
    }

    *last  = lst;
    *skip  = run;
    *value = (level ^ -sign) + sign;

    return 0;
}

static inline int vc1_inverse_quantize_coeff(int coeff, int double_quant, int quant_scale)
{
    int sign;

    if (coeff == 0)
        return 0;

    sign = coeff < 0;
    coeff *= double_quant;
    coeff += (quant_scale ^ -sign) + sign;

    return av_clip(coeff, -2048, 2047);
}

static int vc1_decode_ac_coeff_simple(VC1BlkCtx *blkctx,
                                      int16_t *block,
                                      int block_size,
                                      GetBitContext *gb)
{
    VC1ACCodingSet *coding_set = blkctx->ac_coding_set;
    uint64_t inv_quantize_mask = blkctx->btype == BLOCK_INTRA ? 0xfefefefefefefe01 : UINT64_C(-1);
    unsigned int symbol;
    int ac_diff, level, sign;
    int run, last_flag, escape_mode;
    int i, k;

    i = blkctx->btype == BLOCK_INTRA ? 1 : 0;
    do {
        symbol = get_vlc2(gb, // ESCMODE | ACCOEF1
                          coding_set->index_vlc->table,
                          9,
                          coding_set->max_depth);
        escape_mode = symbol >> 13 & 3;

        switch (escape_mode) {
        case 0:
            run = symbol & 0x3f;
            level = symbol >> 6 & 0x3f;
            last_flag = symbol >> 12 & 1;
            sign = get_bits1(gb); // LVLSGN
            break;

        case 1: // AC Escape Decoding Mode 1
        case 2: // AC Escape Decoding Mode 2
            symbol = get_vlc2(gb, // ACCOEF2
                              coding_set->index_vlc->table,
                              9,
                              coding_set->max_depth);
            if (symbol >> 13 & 3)
                return AVERROR_INVALIDDATA;

            run = symbol & 0x3f;
            level = symbol >> 6 & 0x3f;
            last_flag = symbol >> 12 & 1;

            if (escape_mode == 1)
                level += coding_set->delta_level_table[run + coding_set->delta_level_idx_of_last * last_flag];
            else
                run += coding_set->delta_run_table[level + coding_set->delta_run_idx_of_last * last_flag] + 1;

            sign = get_bits1(gb); // LVLSGN
            break;

        case 3: // AC Escape Decoding Mode 3
            last_flag = get_bits1(gb); // ESCLR

            if (*blkctx->ac_level_code_size == 0) {
                if (blkctx->esc_mode3_vlc) {
                    *blkctx->ac_level_code_size = get_bits(gb, 3); // ESCLVLSZ
                    if (*blkctx->ac_level_code_size == 0)
                        *blkctx->ac_level_code_size = 8 + get_bits(gb, 2); // ESCLVLSZ
                } else {
                    *blkctx->ac_level_code_size = 2 + get_unary(gb, 1, 6); // ESCLVLSZ
                }
                *blkctx->ac_run_code_size = 3 + get_bits(gb, 2); // ESCRUNSZ
            }

            symbol = get_bits(gb, *blkctx->ac_level_code_size + *blkctx->ac_run_code_size + 1); // ESCRUN | LVLSIGN2 | ESCLVL

            level = symbol & (1 << *blkctx->ac_level_code_size) - 1;
            sign = symbol >> *blkctx->ac_level_code_size & 1;
            run = symbol >> *blkctx->ac_level_code_size + 1;
            break;
        }

        i += run;
        if (i > block_size)
            return AVERROR_INVALIDDATA;

        ac_diff = (level ^ -sign) + sign;

        /* Inverse quantize AC coefficients */
        k = blkctx->zz[i++];
        block[k] += ac_diff;
        if (inv_quantize_mask >> k & UINT64_C(1))
            block[k] = vc1_inverse_quantize_coeff(block[k],
                                                  blkctx->double_quant,
                                                  blkctx->quant_scale);
    } while(!last_flag);

    return i;
}

static inline void vc1_predict_intra_coeff(VC1IntraBlkCtx *blkctx,
                                           int curr_blkidx, int top_blkidx,
                                           int topleft_blkidx, int left_blkidx)
{
    VC1StoredBlkCtx *curr_sblkctx = blkctx->s_blkctx + curr_blkidx;
    VC1StoredBlkCtx *pred_a_sblkctx = blkctx->s_blkctx + top_blkidx;
    VC1StoredBlkCtx *pred_b_sblkctx = blkctx->s_blkctx + topleft_blkidx;
    VC1StoredBlkCtx *pred_c_sblkctx = blkctx->s_blkctx + left_blkidx;
    int16_t *block = blkctx->block[curr_blkidx];
    int dc_dqscale = ff_vc1_dqscale[ff_vc1_dc_scale_table[blkctx->mquant]];
    int a_avail = (pred_a_sblkctx->btype & ~BLOCK_OOB) == BLOCK_INTRA;
    int b_avail = (pred_b_sblkctx->btype & ~BLOCK_OOB) == BLOCK_INTRA;
    int c_avail = (pred_c_sblkctx->btype & ~BLOCK_OOB) == BLOCK_INTRA;
    int pred_dir;
    int a, b, c;

    /* B A
     * C X
     */

    /* Predict CBP and store for further prediction */
    if (blkctx->use_cbpcy_pred) {
        a = pred_a_sblkctx->is_coded;
        b = pred_b_sblkctx->is_coded;
        c = pred_c_sblkctx->is_coded;

        curr_sblkctx->is_coded ^= (b ^ a ? a : c);
    }

    /* Predict DC */
    a = a_avail ? (pred_a_sblkctx->dc_pred * dc_dqscale + 0x20000) >> 18 : 0;
    b = b_avail ? (pred_b_sblkctx->dc_pred * dc_dqscale + 0x20000) >> 18 : 0;
    c = c_avail ? (pred_c_sblkctx->dc_pred * dc_dqscale + 0x20000) >> 18 : 0;

    pred_dir = !a_avail || (c_avail && (b - a) * (b - a) <= (b - c) * (b - c)) ?
               PRED_DIR_LEFT :
               PRED_DIR_TOP;
    block[0] = pred_dir == PRED_DIR_LEFT ? c : a;

    /* Predict AC */
    if (blkctx->use_ac_pred && (a_avail || c_avail)) {
        int pred_blk_sh = (pred_dir == PRED_DIR_LEFT) ^ blkctx->fasttx ? 3 : 0;
        int16_t *ac_pred = pred_dir == PRED_DIR_LEFT ?
                           pred_c_sblkctx->ac_pred_left :
                           pred_a_sblkctx->ac_pred_top;
        int ac_dqscale = ff_vc1_dqscale[blkctx->double_quant - 1];

        for (int k = 1; k < 8; k++)
            block[k << pred_blk_sh] = (ac_pred[k - 1] * ac_dqscale + 0x20000) >> 18;
    }

    if (blkctx->use_ac_pred)
        blkctx->zz = *(*blkctx->zz_8x8)[pred_dir == PRED_DIR_LEFT ? 2 : 1];
    else
        blkctx->zz = *(*blkctx->zz_8x8)[0];
}

static int vc1_decode_intra_diff(VC1IntraBlkCtx *blkctx,
                                 int curr_blkidx,
                                 GetBitContext *gb)
{
    VC1StoredBlkCtx *curr_sblkctx = blkctx->s_blkctx + curr_blkidx;
    int16_t *block = blkctx->block[curr_blkidx];
    int mquant = blkctx->mquant;
    int double_quant = blkctx->double_quant;
    int quant_scale = blkctx->quant_scale;
    int fasttx_blk_sh = blkctx->fasttx ? 3 : 0;
    int dc_diff;
    int ret;

    /* Decode DC differential */
    dc_diff = get_vlc2(gb, blkctx->dc_diff_vlc->table, 9, 3); // DCCOEF
    if (dc_diff > 0) {
        int m = (3 - mquant) * (mquant < 3);
        int sign;

        if (dc_diff == 119) {
            dc_diff = get_bits(gb, 8 + m); // DCCOEF_ESC
        } else {
            if (m)
                dc_diff = (dc_diff - 1 << m) + 1 + get_bits(gb, m); // DCCOEF_EXTQUANT[12]
        }

        sign = get_bits1(gb); // DCSIGN
        dc_diff = (dc_diff ^ -sign) + sign;

        block[0] += dc_diff;
    }

    /* Decode AC differentials */
    if (curr_sblkctx->is_coded) {
        ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                         block,
                                         63,
                                         gb);
        if (ret < 0)
            return ret;
    }

    /* Inverse quantize DC coefficient and */
    /* store DC coefficient for further prediction */
    curr_sblkctx->dc_pred =
        block[0] = av_clip(block[0] * ff_vc1_dc_scale_table[mquant], -2048, 2047);

    for (int k = 1; k < 8; k++) {
        /* Store scaled AC coefficients for further prediction */
        curr_sblkctx->ac_pred_top[k - 1] =
            av_clip(block[k << fasttx_blk_sh] * (double_quant - 1), -2048, 2047);
        curr_sblkctx->ac_pred_left[k - 1] =
            av_clip(block[k << (fasttx_blk_sh ^ 3)] * (double_quant - 1), -2048, 2047);

        /* Inverse quantize AC coefficients */
        block[k] = vc1_inverse_quantize_coeff(block[k],
                                              double_quant,
                                              quant_scale);
        block[k << 3] = vc1_inverse_quantize_coeff(block[k << 3],
                                                   double_quant,
                                                   quant_scale);
    }

    return 0;
}

static int vc1_decode_intra_block_new(VC1MBCtx* mbctx,
                                      VC1IntraBlkCtx *blkctx,
                                      int curr_mbidx,
                                      int curr_blkidx, int top_blkidx,
                                      int topleft_blkidx, int left_blkidx,
                                      GetBitContext *gb)
{
    VC1StoredBlkCtx *sblkctx = blkctx->s_blkctx;
    VC1StoredBlkCtx *curr_sblkctx = blkctx->s_blkctx + curr_blkidx;
    int16_t (*block)[64] = blkctx->block;
    unsigned int top_btype = sblkctx[top_blkidx].btype;
    unsigned int left_btype = sblkctx[left_blkidx].btype;
    int ret;

    curr_sblkctx->btype = BLOCK_INTRA;

    vc1_predict_intra_coeff(blkctx, curr_blkidx, top_blkidx, topleft_blkidx, left_blkidx);

    ret = vc1_decode_intra_diff(blkctx, curr_blkidx, gb);
    if (ret < 0)
        return ret;

    if (!CONFIG_GRAY || !blkctx->skip_output)
        blkctx->vc1dsp->vc1_inv_trans_8x8(block[curr_blkidx]);

    // TODO: this should be an always inlined fucntion in vc1_loopback.c
    if (mbctx->use_overlap_xfrm)
        curr_sblkctx->overlap = left_btype == BLOCK_INTRA |
                                       (top_btype == BLOCK_INTRA) << 1;

    // TODO: this should be an always inlined fucntion in vc1_loopback.c
    if (mbctx->use_loopfilter) {
        curr_sblkctx->loopfilter = LOOPFILTER_OUTER;

        mbctx->s_mbctx[curr_mbidx].loopfilter_blk <<= 4;
        mbctx->s_mbctx[curr_mbidx].loopfilter_subblk <<= 4;

        if ((top_btype & BLOCK_OOB) != BLOCK_OOB)
            mbctx->s_mbctx[curr_mbidx].loopfilter_blk |= (LOOPFILTER_OUTER & LOOPFILTER_TOP_MASK) >> LOOPFILTER_TOPLEFT_SHIFT;

        if ((left_btype & BLOCK_OOB) != BLOCK_OOB)
            mbctx->s_mbctx[curr_mbidx].loopfilter_blk |= (LOOPFILTER_OUTER & LOOPFILTER_LEFT_MASK) >> LOOPFILTER_TOPLEFT_SHIFT;
    }

    return 0;
}

/** Decode intra block in intra frames - should be faster than decode_intra_block
 * @param v VC1Context
 * @param block block to decode
 * @param[in] n subblock number
 * @param coded are AC coeffs present or not
 * @param codingset set of VLC to decode data
 * @param mquant quantizer value for this macroblock
 */
static int vc1_decode_i_block_adv(VC1Context *v, int16_t block[64], int n,
                                  int coded, int codingset, int mquant)
{
    GetBitContext *gb = &v->s.gb;
    MpegEncContext *s = &v->s;
    int dc_pred_dir = 0; /* Direction of the DC prediction used */
    int i;
    int16_t *dc_val = NULL;
    int16_t *ac_val, *ac_val2;
    int dcdiff;
    int a_avail = v->a_avail, c_avail = v->c_avail;
    int use_pred = s->ac_pred;
    int scale;
    int q1, q2 = 0;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int quant = FFABS(mquant);

    /* Get DC differential */
    if (n < 4) {
        dcdiff = get_vlc2(&s->gb, ff_msmp4_dc_luma_vlc[s->dc_table_index].table, DC_VLC_BITS, 3);
    } else {
        dcdiff = get_vlc2(&s->gb, ff_msmp4_dc_chroma_vlc[s->dc_table_index].table, DC_VLC_BITS, 3);
    }
    if (dcdiff < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Illegal DC VLC\n");
        return -1;
    }
    if (dcdiff) {
        const int m = (quant == 1 || quant == 2) ? 3 - quant : 0;
        if (dcdiff == 119 /* ESC index value */) {
            dcdiff = get_bits(gb, 8 + m);
        } else {
            if (m)
                dcdiff = (dcdiff << m) + get_bits(gb, m) - ((1 << m) - 1);
        }
        if (get_bits1(gb))
            dcdiff = -dcdiff;
    }

    /* Prediction */
    dcdiff += ff_vc1_pred_dc(&v->s, v->overlap, quant, n, v->a_avail, v->c_avail, &dc_val, &dc_pred_dir);
    *dc_val = dcdiff;

    /* Store the quantized DC coeff, used for prediction */
    if (n < 4)
        scale = s->y_dc_scale;
    else
        scale = s->c_dc_scale;
    block[0] = dcdiff * scale;

    /* check if AC is needed at all */
    if (!a_avail && !c_avail)
        use_pred = 0;

    scale = quant * 2 + ((mquant < 0) ? 0 : v->halfpq);

    ac_val  = s->ac_val[0][s->block_index[n]];
    ac_val2 = ac_val;
    if (dc_pred_dir) // left
        ac_val -= 16;
    else // top
        ac_val -= 16 * s->block_wrap[n];

    q1 = s->current_picture.qscale_table[mb_pos];
    if (n == 3)
        q2 = q1;
    else if (dc_pred_dir) {
        if (n == 1)
            q2 = q1;
        else if (c_avail && mb_pos)
            q2 = s->current_picture.qscale_table[mb_pos - 1];
    } else {
        if (n == 2)
            q2 = q1;
        else if (a_avail && mb_pos >= s->mb_stride)
            q2 = s->current_picture.qscale_table[mb_pos - s->mb_stride];
    }

    //AC Decoding
    i = 1;

    if (coded) {
        int last = 0, skip, value;
        const uint8_t *zz_table;
        int k;

        if (v->s.ac_pred) {
            if (!use_pred && v->fcm == ILACE_FRAME) {
                zz_table = v->zzi_8x8;
            } else {
                if (!dc_pred_dir) // top
                    zz_table = v->zz_8x8[2];
                else // left
                    zz_table = v->zz_8x8[3];
            }
        } else {
            if (v->fcm != ILACE_FRAME)
                zz_table = v->zz_8x8[1];
            else
                zz_table = v->zzi_8x8;
        }

        while (!last) {
            int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, codingset);
            if (ret < 0)
                return ret;
            i += skip;
            if (i > 63)
                break;
            block[zz_table[i++]] = value;
        }

        /* apply AC prediction if needed */
        if (use_pred) {
            int sh;
            if (dc_pred_dir) { // left
                sh = v->left_blk_sh;
            } else { // top
                sh = v->top_blk_sh;
                ac_val += 8;
            }
            /* scale predictors if needed*/
            q1 = FFABS(q1) * 2 + ((q1 < 0) ? 0 : v->halfpq) - 1;
            if (q1 < 1)
                return AVERROR_INVALIDDATA;
            if (q2)
                q2 = FFABS(q2) * 2 + ((q2 < 0) ? 0 : v->halfpq) - 1;
            if (q2 && q1 != q2) {
                for (k = 1; k < 8; k++)
                    block[k << sh] += (int)(ac_val[k] * (unsigned)q2 * ff_vc1_dqscale[q1] + 0x20000) >> 18;
            } else {
                for (k = 1; k < 8; k++)
                    block[k << sh] += ac_val[k];
            }
        }
        /* save AC coeffs for further prediction */
        for (k = 1; k < 8; k++) {
            ac_val2[k    ] = block[k << v->left_blk_sh];
            ac_val2[k + 8] = block[k << v->top_blk_sh];
        }

        /* scale AC coeffs */
        for (k = 1; k < 64; k++)
            if (block[k]) {
                block[k] *= scale;
                if (!v->pquantizer)
                    block[k] += (block[k] < 0) ? -quant : quant;
            }

    } else { // no AC coeffs
        int k;

        memset(ac_val2, 0, 16 * 2);

        /* apply AC prediction if needed */
        if (use_pred) {
            int sh;
            if (dc_pred_dir) { // left
                sh = v->left_blk_sh;
            } else { // top
                sh = v->top_blk_sh;
                ac_val  += 8;
                ac_val2 += 8;
            }
            memcpy(ac_val2, ac_val, 8 * 2);
            q1 = FFABS(q1) * 2 + ((q1 < 0) ? 0 : v->halfpq) - 1;
            if (q1 < 1)
                return AVERROR_INVALIDDATA;
            if (q2)
                q2 = FFABS(q2) * 2 + ((q2 < 0) ? 0 : v->halfpq) - 1;
            if (q2 && q1 != q2) {
                for (k = 1; k < 8; k++)
                    ac_val2[k] = (int)(ac_val2[k] * q2 * (unsigned)ff_vc1_dqscale[q1] + 0x20000) >> 18;
            }
            for (k = 1; k < 8; k++) {
                block[k << sh] = ac_val2[k] * scale;
                if (!v->pquantizer && block[k << sh])
                    block[k << sh] += (block[k << sh] < 0) ? -quant : quant;
            }
        }
    }
    if (use_pred) i = 63;
    s->block_last_index[n] = i;

    return 0;
}

/** Decode intra block in inter frames - more generic version than vc1_decode_i_block
 * @param v VC1Context
 * @param block block to decode
 * @param[in] n subblock index
 * @param coded are AC coeffs present or not
 * @param mquant block quantizer
 * @param codingset set of VLC to decode data
 */
static int vc1_decode_intra_block(VC1Context *v,
                                  int16_t block[64],
                                  int n,
                                  int coded,
                                  int mquant,
                                  int codingset)
{
    GetBitContext *gb = &v->s.gb;
    MpegEncContext *s = &v->s;
    int dc_pred_dir = 0; /* Direction of the DC prediction used */
    int i;
    int16_t *dc_val = NULL;
    int16_t *ac_val, *ac_val2;
    int dcdiff;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int a_avail = v->a_avail, c_avail = v->c_avail;
    int use_pred = s->ac_pred;
    int scale;
    int q1, q2 = 0;
    int quant = FFABS(mquant);
    int dcdiff_old;

    s->bdsp.clear_block(block);

    /* XXX: Guard against dumb values of mquant */
    quant = av_clip_uintp2(quant, 5);

    /* Set DC scale - y and c use the same */
    s->y_dc_scale = s->y_dc_scale_table[quant];
    s->c_dc_scale = s->c_dc_scale_table[quant];

    /* Get DC differential */
    if (n < 4) {
        dcdiff = get_vlc2(&s->gb, ff_msmp4_dc_luma_vlc[s->dc_table_index].table, DC_VLC_BITS, 3);
    } else {
        dcdiff = get_vlc2(&s->gb, ff_msmp4_dc_chroma_vlc[s->dc_table_index].table, DC_VLC_BITS, 3);
    }
    if (dcdiff < 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Illegal DC VLC\n");
        return -1;
    }
    if (dcdiff) {
        const int m = (quant == 1 || quant == 2) ? 3 - quant : 0;
        if (dcdiff == 119 /* ESC index value */) {
            dcdiff = get_bits(gb, 8 + m);
        } else {
            if (m)
                dcdiff = (dcdiff << m) + get_bits(gb, m) - ((1 << m) - 1);
        }
        if (get_bits1(gb))
            dcdiff = -dcdiff;
    }

    /* Prediction */
    dcdiff_old = ff_vc1_pred_dc(&v->s, v->overlap, quant, n, a_avail, c_avail, &dc_val, &dc_pred_dir);
    dcdiff += dcdiff_old;
    *dc_val = dcdiff;

    /* Store the quantized DC coeff, used for prediction */

    if (n < 4) {
        block[0] = dcdiff * s->y_dc_scale;
    } else {
        block[0] = dcdiff * s->c_dc_scale;
    }

    //AC Decoding
    i = 1;

    /* check if AC is needed at all and adjust direction if needed */
    if (!a_avail) dc_pred_dir = 1;
    if (!c_avail) dc_pred_dir = 0;
    if (!a_avail && !c_avail) use_pred = 0;
    ac_val = s->ac_val[0][s->block_index[n]];
    ac_val2 = ac_val;

    scale = quant * 2 + ((mquant < 0) ? 0 : v->halfpq);

    if (dc_pred_dir) //left
        ac_val -= 16;
    else //top
        ac_val -= 16 * s->block_wrap[n];

    q1 = s->current_picture.qscale_table[mb_pos];
    if (dc_pred_dir && c_avail && mb_pos)
        q2 = s->current_picture.qscale_table[mb_pos - 1];
    if (!dc_pred_dir && a_avail && mb_pos >= s->mb_stride)
        q2 = s->current_picture.qscale_table[mb_pos - s->mb_stride];
    if (dc_pred_dir && n == 1)
        q2 = q1;
    if (!dc_pred_dir && n == 2)
        q2 = q1;
    if (n == 3) q2 = q1;

    if (coded) {
        int last = 0, skip, value;
        int k;

        while (!last) {
            int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, codingset);
            if (ret < 0)
                return ret;
            i += skip;
            if (i > 63)
                break;
            if (v->fcm == PROGRESSIVE)
                block[v->zz_8x8[0][i++]] = value;
            else {
                if (use_pred && (v->fcm == ILACE_FRAME)) {
                    if (!dc_pred_dir) // top
                        block[v->zz_8x8[2][i++]] = value;
                    else // left
                        block[v->zz_8x8[3][i++]] = value;
                } else {
                    block[v->zzi_8x8[i++]] = value;
                }
            }
        }

        /* apply AC prediction if needed */
        if (use_pred) {
            /* scale predictors if needed*/
            q1 = FFABS(q1) * 2 + ((q1 < 0) ? 0 : v->halfpq) - 1;
            if (q1 < 1)
                return AVERROR_INVALIDDATA;
            if (q2)
                q2 = FFABS(q2) * 2 + ((q2 < 0) ? 0 : v->halfpq) - 1;
            if (q2 && q1 != q2) {
                if (dc_pred_dir) { // left
                    for (k = 1; k < 8; k++)
                        block[k << v->left_blk_sh] += (int)(ac_val[k] * q2 * (unsigned)ff_vc1_dqscale[q1] + 0x20000) >> 18;
                } else { //top
                    for (k = 1; k < 8; k++)
                        block[k << v->top_blk_sh] += (int)(ac_val[k + 8] * q2 * (unsigned)ff_vc1_dqscale[q1] + 0x20000) >> 18;
                }
            } else {
                if (dc_pred_dir) { // left
                    for (k = 1; k < 8; k++)
                        block[k << v->left_blk_sh] += ac_val[k];
                } else { // top
                    for (k = 1; k < 8; k++)
                        block[k << v->top_blk_sh] += ac_val[k + 8];
                }
            }
        }
        /* save AC coeffs for further prediction */
        for (k = 1; k < 8; k++) {
            ac_val2[k    ] = block[k << v->left_blk_sh];
            ac_val2[k + 8] = block[k << v->top_blk_sh];
        }

        /* scale AC coeffs */
        for (k = 1; k < 64; k++)
            if (block[k]) {
                block[k] *= scale;
                if (!v->pquantizer)
                    block[k] += (block[k] < 0) ? -quant : quant;
            }

        if (use_pred) i = 63;
    } else { // no AC coeffs
        int k;

        memset(ac_val2, 0, 16 * 2);
        if (dc_pred_dir) { // left
            if (use_pred) {
                memcpy(ac_val2, ac_val, 8 * 2);
                q1 = FFABS(q1) * 2 + ((q1 < 0) ? 0 : v->halfpq) - 1;
                if (q1 < 1)
                    return AVERROR_INVALIDDATA;
                if (q2)
                    q2 = FFABS(q2) * 2 + ((q2 < 0) ? 0 : v->halfpq) - 1;
                if (q2 && q1 != q2) {
                    for (k = 1; k < 8; k++)
                        ac_val2[k] = (ac_val2[k] * q2 * ff_vc1_dqscale[q1] + 0x20000) >> 18;
                }
            }
        } else { // top
            if (use_pred) {
                memcpy(ac_val2 + 8, ac_val + 8, 8 * 2);
                q1 = FFABS(q1) * 2 + ((q1 < 0) ? 0 : v->halfpq) - 1;
                if (q1 < 1)
                    return AVERROR_INVALIDDATA;
                if (q2)
                    q2 = FFABS(q2) * 2 + ((q2 < 0) ? 0 : v->halfpq) - 1;
                if (q2 && q1 != q2) {
                    for (k = 1; k < 8; k++)
                        ac_val2[k + 8] = (ac_val2[k + 8] * q2 * ff_vc1_dqscale[q1] + 0x20000) >> 18;
                }
            }
        }

        /* apply AC prediction if needed */
        if (use_pred) {
            if (dc_pred_dir) { // left
                for (k = 1; k < 8; k++) {
                    block[k << v->left_blk_sh] = ac_val2[k] * scale;
                    if (!v->pquantizer && block[k << v->left_blk_sh])
                        block[k << v->left_blk_sh] += (block[k << v->left_blk_sh] < 0) ? -quant : quant;
                }
            } else { // top
                for (k = 1; k < 8; k++) {
                    block[k << v->top_blk_sh] = ac_val2[k + 8] * scale;
                    if (!v->pquantizer && block[k << v->top_blk_sh])
                        block[k << v->top_blk_sh] += (block[k << v->top_blk_sh] < 0) ? -quant : quant;
                }
            }
            i = 63;
        }
    }
    s->block_last_index[n] = i;

    return 0;
}

/** Decode P block
 */
static int vc1_decode_p_block(VC1Context *v, int16_t block[64], int n,
                              int mquant, int ttmb, int first_block,
                              uint8_t *dst, int linesize, int skip_block,
                              int *ttmb_out)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i, j;
    int subblkpat = 0;
    int scale, off, idx, last, skip, value;
    int ttblk = ttmb & 7;
    int pat = 0;
    int quant = FFABS(mquant);

    s->bdsp.clear_block(block);

    if (ttmb == -1) {
        ttblk = ff_vc1_ttblk_to_tt[v->tt_index][get_vlc2(gb, ff_vc1_ttblk_vlc[v->tt_index].table, VC1_TTBLK_VLC_BITS, 1)];
    }
    if (ttblk == TT_4X4) {
        subblkpat = ~(get_vlc2(gb, ff_vc1_subblkpat_vlc[v->tt_index].table, VC1_SUBBLKPAT_VLC_BITS, 1) + 1);
    }
    if ((ttblk != TT_8X8 && ttblk != TT_4X4)
        && ((v->ttmbf || (ttmb != -1 && (ttmb & 8) && !first_block))
            || ((v->seq->profile < PROFILE_ADVANCED && !((VC1SimpleSeqCtx*)v->seq)->res_rtm_flag) && !first_block))) {
        subblkpat = decode012(gb);
        if (subblkpat)
            subblkpat ^= 3; // swap decoded pattern bits
        if (ttblk == TT_8X4_TOP || ttblk == TT_8X4_BOTTOM)
            ttblk = TT_8X4;
        if (ttblk == TT_4X8_RIGHT || ttblk == TT_4X8_LEFT)
            ttblk = TT_4X8;
    }
    scale = quant * 2 + ((mquant < 0) ? 0 : v->halfpq);

    // convert transforms like 8X4_TOP to generic TT and SUBBLKPAT
    if (ttblk == TT_8X4_TOP || ttblk == TT_8X4_BOTTOM) {
        subblkpat = 2 - (ttblk == TT_8X4_TOP);
        ttblk     = TT_8X4;
    }
    if (ttblk == TT_4X8_RIGHT || ttblk == TT_4X8_LEFT) {
        subblkpat = 2 - (ttblk == TT_4X8_LEFT);
        ttblk     = TT_4X8;
    }


    switch (ttblk) {
    case TT_8X8:
        pat  = 0xF;
        i    = 0;
        last = 0;
        while (!last) {
            int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, v->codingset2);
            if (ret < 0)
                return ret;
            i += skip;
            if (i > 63)
                break;
            if (!v->fcm)
                idx = v->zz_8x8[0][i++];
            else
                idx = v->zzi_8x8[i++];
            block[idx] = value * scale;
            if (!v->pquantizer)
                block[idx] += (block[idx] < 0) ? -quant : quant;
        }
        if (!skip_block) {
            if (i == 1)
                v->vc1dsp.vc1_inv_trans_8x8_dc(dst, linesize, block);
            else {
                v->vc1dsp.vc1_inv_trans_8x8(block);
                s->idsp.add_pixels_clamped(block, dst, linesize);
            }
        }
        break;
    case TT_4X4:
        pat = ~subblkpat & 0xF;
        for (j = 0; j < 4; j++) {
            last = subblkpat & (1 << (3 - j));
            i    = 0;
            off  = (j & 1) * 4 + (j & 2) * 16;
            while (!last) {
                int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, v->codingset2);
                if (ret < 0)
                    return ret;
                i += skip;
                if (i > 15)
                    break;
                if (!v->fcm)
                    idx = ff_vc1_simple_progressive_4x4_zz[i++];
                else
                    idx = ff_vc1_adv_interlaced_4x4_zz[i++];
                block[idx + off] = value * scale;
                if (!v->pquantizer)
                    block[idx + off] += (block[idx + off] < 0) ? -quant : quant;
            }
            if (!(subblkpat & (1 << (3 - j))) && !skip_block) {
                if (i == 1)
                    v->vc1dsp.vc1_inv_trans_4x4_dc(dst + (j & 1) * 4 + (j & 2) * 2 * linesize, linesize, block + off);
                else
                    v->vc1dsp.vc1_inv_trans_4x4(dst + (j & 1) * 4 + (j & 2) *  2 * linesize, linesize, block + off);
            }
        }
        break;
    case TT_8X4:
        pat = ~((subblkpat & 2) * 6 + (subblkpat & 1) * 3) & 0xF;
        for (j = 0; j < 2; j++) {
            last = subblkpat & (1 << (1 - j));
            i    = 0;
            off  = j * 32;
            while (!last) {
                int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, v->codingset2);
                if (ret < 0)
                    return ret;
                i += skip;
                if (i > 31)
                    break;
                if (!v->fcm)
                    idx = v->zz_8x4[i++] + off;
                else
                    idx = ff_vc1_adv_interlaced_8x4_zz[i++] + off;
                block[idx] = value * scale;
                if (!v->pquantizer)
                    block[idx] += (block[idx] < 0) ? -quant : quant;
            }
            if (!(subblkpat & (1 << (1 - j))) && !skip_block) {
                if (i == 1)
                    v->vc1dsp.vc1_inv_trans_8x4_dc(dst + j * 4 * linesize, linesize, block + off);
                else
                    v->vc1dsp.vc1_inv_trans_8x4(dst + j * 4 * linesize, linesize, block + off);
            }
        }
        break;
    case TT_4X8:
        pat = ~(subblkpat * 5) & 0xF;
        for (j = 0; j < 2; j++) {
            last = subblkpat & (1 << (1 - j));
            i    = 0;
            off  = j * 4;
            while (!last) {
                int ret = vc1_decode_ac_coeff(v, &last, &skip, &value, v->codingset2);
                if (ret < 0)
                    return ret;
                i += skip;
                if (i > 31)
                    break;
                if (!v->fcm)
                    idx = v->zz_4x8[i++] + off;
                else
                    idx = ff_vc1_adv_interlaced_4x8_zz[i++] + off;
                block[idx] = value * scale;
                if (!v->pquantizer)
                    block[idx] += (block[idx] < 0) ? -quant : quant;
            }
            if (!(subblkpat & (1 << (1 - j))) && !skip_block) {
                if (i == 1)
                    v->vc1dsp.vc1_inv_trans_4x8_dc(dst + j * 4, linesize, block + off);
                else
                    v->vc1dsp.vc1_inv_trans_4x8(dst + j*4, linesize, block + off);
            }
        }
        break;
    }
    if (ttmb_out)
        *ttmb_out |= ttblk << (n * 4);
    return pat;
}

static int vc1_decode_inter_block(VC1InterBlkCtx *blkctx,
                                  int curr_blkidx,
                                  GetBitContext *gb)
{
    static const uint16_t loopfilter_4x4[16] = {
        LOOPFILTER_NONE,
        LOOPFILTER_SB3,
        LOOPFILTER_SB2,
        LOOPFILTER_SB2 | LOOPFILTER_SB3,
        LOOPFILTER_SB1,
        LOOPFILTER_SB1 | LOOPFILTER_SB3,
        LOOPFILTER_SB1 | LOOPFILTER_SB2,
        LOOPFILTER_SB1 | LOOPFILTER_SB2 | LOOPFILTER_SB3,
        LOOPFILTER_SB0,
        LOOPFILTER_SB0 | LOOPFILTER_SB3,
        LOOPFILTER_SB0 | LOOPFILTER_SB2,
        LOOPFILTER_SB0 | LOOPFILTER_SB2 | LOOPFILTER_SB3,
        LOOPFILTER_SB0 | LOOPFILTER_SB1,
        LOOPFILTER_SB0 | LOOPFILTER_SB1 | LOOPFILTER_SB3,
        LOOPFILTER_SB0 | LOOPFILTER_SB1 | LOOPFILTER_SB2,
        LOOPFILTER_SB0 | LOOPFILTER_SB1 | LOOPFILTER_SB2 | LOOPFILTER_SB3
    };

    VC1DSPContext *vc1dsp = blkctx->vc1dsp;
    int16_t *block = blkctx->block[curr_blkidx];
    uint8_t *dest = blkctx->dest;
    int linesize = blkctx->linesize;
    unsigned int loopfilter = LOOPFILTER_NONE;
    int subblkpat;
    int ret;

    if (!blkctx->tt) {
        blkctx->tt = get_vlc2(gb, blkctx->ttblk_vlc->table, 5, 1); // TTBLK
        if (!blkctx->res_rtm_flag)
            blkctx->tt &= ~SUBBLOCK_MASK;
    }

    switch ((blkctx->tt & TT_MASK) >> 5) {
    case TT_8x8_new >> 5:
        blkctx->zz = ff_vc1_inter_8x8_scan_zz_table;

        subblkpat = SUBBLOCK_ALL;

        ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                         block,
                                         63,
                                         gb);
        if (ret < 0)
            return ret;

        if (!CONFIG_GRAY || !blkctx->skip_output) {
            if (ret == 1)
                vc1dsp->vc1_inv_trans_8x8_dc(dest, linesize, block);
            else {
                vc1dsp->vc1_inv_trans_8x8(block);
                blkctx->idsp->add_pixels_clamped(block, dest, linesize);
            }
        }

        loopfilter = LOOPFILTER_OUTER;

        break;

    case TT_4x4_new >> 5:
        blkctx->zz = ff_vc1_inter_4x4_scan_zz_table;

        subblkpat = get_vlc2(gb, blkctx->subblkpat_vlc->table, 6, 1) + 1; // SUBBLKPAT
        if (subblkpat & SUBBLOCK_SB0) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block,
                                             15,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x4_dc(dest, linesize, block);
                else
                    vc1dsp->vc1_inv_trans_4x4(dest, linesize, block);
            }
        }
        if (subblkpat & SUBBLOCK_SB1) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block + 4,
                                             15,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x4_dc(dest + 4, linesize, block + 4);
                else
                    vc1dsp->vc1_inv_trans_4x4(dest + 4, linesize, block + 4);
            }
        }
        if (subblkpat & SUBBLOCK_SB2) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block + 32,
                                             15,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x4_dc(dest + 4 * linesize, linesize, block + 32);
                else
                    vc1dsp->vc1_inv_trans_4x4(dest + 4 * linesize, linesize, block + 32);
            }
        }
        if (subblkpat & SUBBLOCK_SB3) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block + 36,
                                             15,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x4_dc(dest + 4 + 4 * linesize, linesize, block + 36);
                else
                    vc1dsp->vc1_inv_trans_4x4(dest + 4 + 4 * linesize, linesize, block + 36);
            }
        }

        loopfilter = loopfilter_4x4[subblkpat];

        break;

    case TT_8x4_new >> 5:
        blkctx->zz = ff_vc1_inter_8x4_scan_zz_table;

        subblkpat = blkctx->tt & SUBBLOCK_MASK;
        if (!subblkpat) {
            static const uint8_t subblkpat_8x4[3] = { SUBBLOCK_BOTH, SUBBLOCK_BOTTOM, SUBBLOCK_TOP };

            subblkpat = subblkpat_8x4[decode012(gb)]; // SUBBLKPAT
        }
        if (subblkpat & SUBBLOCK_TOP) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block,
                                             31,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_8x4_dc(dest, linesize, block);
                else
                    vc1dsp->vc1_inv_trans_8x4(dest, linesize, block);
            }

            loopfilter |= LOOPFILTER_TOP;
        }
        if (subblkpat & SUBBLOCK_BOTTOM) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block + 32,
                                             31,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_8x4_dc(dest + 4 * linesize, linesize, block + 32);
                else
                    vc1dsp->vc1_inv_trans_8x4(dest + 4 * linesize, linesize, block + 32);
            }

            loopfilter |= LOOPFILTER_BOTTOM;
        }

        break;

    case TT_4x8_new >> 5:
        blkctx->zz = ff_vc1_inter_4x8_scan_zz_table;

        subblkpat = blkctx->tt & SUBBLOCK_MASK;
        if (!subblkpat) {
            static const uint8_t subblkpat_4x8[3] = { SUBBLOCK_BOTH, SUBBLOCK_RIGHT, SUBBLOCK_LEFT };

            subblkpat = subblkpat_4x8[decode012(gb)]; // SUBBLKPAT
        }
        if (subblkpat & SUBBLOCK_LEFT) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block,
                                             31,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x8_dc(dest, linesize, block);
                else
                    vc1dsp->vc1_inv_trans_4x8(dest, linesize, block);
            }

            loopfilter |= LOOPFILTER_LEFT;
        }
        if (subblkpat & SUBBLOCK_RIGHT) {
            ret = vc1_decode_ac_coeff_simple((VC1BlkCtx*)blkctx,
                                             block + 4,
                                             31,
                                             gb);
            if (ret < 0)
                return ret;

            if (!CONFIG_GRAY || !blkctx->skip_output) {
                if (ret == 1)
                    vc1dsp->vc1_inv_trans_4x8_dc(dest + 4, linesize, block + 4);
                else
                    vc1dsp->vc1_inv_trans_4x8(dest + 4, linesize, block + 4);
            }

            loopfilter |= LOOPFILTER_RIGHT;
        }

        break;

    default:
        av_assert0(0);
    }

    blkctx->s_blkctx[curr_blkidx].loopfilter |= loopfilter;

    if (blkctx->tt & SIGNALLEVEL_MB)
        blkctx->tt &= ~SUBBLOCK_MASK;
    else
        blkctx->tt = 0;

    return 0;
}

static int vc1_decode_p_block_new(VC1PMBCtx *mbctx,
                                  VC1BlkCtx *blkctx,
                                  int curr_mbidx,
                                  int curr_blkidx, int top_blkidx,
                                  int topleft_blkidx, int left_blkidx,
                                  GetBitContext *gb)
{
    VC1StoredMBCtx *curr_smbctx = mbctx->s_mbctx + curr_mbidx;
    VC1StoredBlkCtx *curr_sblkctx = mbctx->s_blkctx + curr_blkidx;
    VC1StoredBlkCtx *top_sblkctx = mbctx->s_blkctx + top_blkidx;
    VC1StoredBlkCtx *left_sblkctx = mbctx->s_blkctx + left_blkidx;
    int ret;

    curr_sblkctx->btype = blkctx->btype;
    curr_sblkctx->overlap = 0;
    curr_sblkctx->loopfilter = LOOPFILTER_NONE;

    switch (blkctx->btype) {
    case BLOCK_INTER:
    case BLOCK_SKIPPED:
        if (curr_sblkctx->is_coded) {
            ret = vc1_decode_inter_block((VC1InterBlkCtx*)blkctx, curr_blkidx, gb);
            if (ret < 0)
                return ret;
        }
        break;

    default:
        av_assert0(0);
    }

    if (mbctx->use_loopfilter) {
        if (top_sblkctx->mv[MV_CTX_FORWARD][MV_X] != curr_sblkctx->mv[MV_CTX_FORWARD][MV_X] ||
            top_sblkctx->mv[MV_CTX_FORWARD][MV_Y] != curr_sblkctx->mv[MV_CTX_FORWARD][MV_Y])
            curr_sblkctx->loopfilter |= LOOPFILTER_TOP_MASK;

        if (left_sblkctx->mv[MV_CTX_FORWARD][MV_X] != curr_sblkctx->mv[MV_CTX_FORWARD][MV_X] ||
            left_sblkctx->mv[MV_CTX_FORWARD][MV_Y] != curr_sblkctx->mv[MV_CTX_FORWARD][MV_Y])
            curr_sblkctx->loopfilter |= LOOPFILTER_LEFT_MASK;

        curr_smbctx->loopfilter_blk <<= 4;
        curr_smbctx->loopfilter_subblk <<= 4;

        if ((top_sblkctx->btype & BLOCK_OOB) != BLOCK_OOB)
            curr_smbctx->loopfilter_blk |= (top_sblkctx->loopfilter & LOOPFILTER_BOTTOM_MASK) >> LOOPFILTER_BOTTOMRIGHT_SHIFT |
                                                         (curr_sblkctx->loopfilter & LOOPFILTER_TOP_MASK) >> LOOPFILTER_TOPLEFT_SHIFT;
        if ((left_sblkctx->btype & BLOCK_OOB) != BLOCK_OOB)
            curr_smbctx->loopfilter_blk |= (left_sblkctx->loopfilter & LOOPFILTER_RIGHT_MASK) >> LOOPFILTER_BOTTOMRIGHT_SHIFT |
                                                         (curr_sblkctx->loopfilter & LOOPFILTER_LEFT_MASK) >> LOOPFILTER_TOPLEFT_SHIFT;
        curr_smbctx->loopfilter_subblk |= (curr_sblkctx->loopfilter & LOOPFILTER_INNER_MASK) >> LOOPFILTER_INNER_SHIFT;
    }

    return 0;
}

/** @} */ // Macroblock group

static const uint8_t size_table[6] = { 0, 2, 3, 4,  5,  8 };

static int vc1_decode_i_mb(VC1IMBCtx *mbctx,
                           VC1IntraBlkCtx *blkctx,
                           int blkidx[BLOCKIDX_MAX],
                           GetBitContext *gb);

/** Decode one P-frame MB
 */
static int vc1_decode_p_mb_new(VC1Context *v,
                               VC1PMBCtx *mbctx,
                               VC1IntraBlkCtx *intra_blkctx,
                               VC1InterBlkCtx *inter_blkctx,
                               int blkidx[BLOCKIDX_MAX],
                               GetBitContext *gb)
{
    VC1StoredBlkCtx *curr_sblkctx = mbctx->s_blkctx + blkidx[BLOCKIDX_Y0];
    VC1BlkCtx skipped_blkctx = { .btype = BLOCK_SKIPPED };
    unsigned int cbpcy = 0;
    int mvmodebit, skipmbbit;
    int pred_b_blkidx;
    int ret;

    // TODO: this changes the crc for SA10164, SA10165 and SA10166
    // However, these streams are Pan and Scan and currently not
    // decoded correctly anyway.
    v->s.bdsp.clear_blocks(mbctx->block[blkidx[BLOCKIDX_Y0]]);

    mvmodebit = v->mv_type_is_raw ? get_bits1(gb) : v->mv_type_mb_plane[v->s.mb_x + v->s.mb_y * v->s.mb_stride]; // MVMODEBIT
    skipmbbit = v->skip_is_raw ? get_bits1(gb) : v->s.mbskip_table[v->s.mb_x + v->s.mb_y * v->s.mb_stride]; // SKIPMBBIT

    mbctx->mvmode = mvmodebit ? MV_MODE_MIXED_MV | MV_MODE_BIT : mbctx->mvmode & ~MV_MODE_BIT;

    if (!skipmbbit)
        cbpcy = mvmodebit ? get_vlc2(gb, mbctx->cbpcy_vlc->table, 8, 2) : 1 << 5; // CBPCY

    curr_sblkctx[0].is_coded = cbpcy >> 5 & 1;
    pred_b_blkidx = blkidx[mvmodebit ? BLOCKIDX_LT3 : BLOCKIDX_RT2];
    if (pred_b_blkidx == -1)
        pred_b_blkidx = blkidx[mvmodebit ? BLOCKIDX_T3 : BLOCKIDX_LT3];

    inter_blkctx->blkoffset_qpel[MV_X] = mbctx->mboffset_qpel[MV_X];
    inter_blkctx->blkoffset_qpel[MV_Y] = mbctx->mboffset_qpel[MV_Y];

    vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0], gb);
    ff_vc1_predict_mv(mbctx,
                      inter_blkctx,
                      blkidx[BLOCKIDX_Y0],
                      blkidx[BLOCKIDX_T2],
                      pred_b_blkidx,
                      blkidx[BLOCKIDX_L1],
                      gb);

    if (mvmodebit) {
        curr_sblkctx[2].is_coded = cbpcy >> 4 & 1;
        inter_blkctx->blkoffset_qpel[MV_X] += 32;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 2, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_T3],
                          blkidx[BLOCKIDX_RT2] == -1 ? blkidx[BLOCKIDX_T2] : blkidx[BLOCKIDX_RT2],
                          blkidx[BLOCKIDX_Y0],
                          gb);

        curr_sblkctx[1].is_coded = cbpcy >> 3 & 1;
        inter_blkctx->blkoffset_qpel[MV_X] -= 32;
        inter_blkctx->blkoffset_qpel[MV_Y] += 32;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 1, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 1,
                          blkidx[BLOCKIDX_Y0],
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_L3],
                          gb);

        curr_sblkctx[3].is_coded = cbpcy >> 2 & 1;
        inter_blkctx->blkoffset_qpel[MV_X] += 32;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 3, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 3,
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_Y0],
                          blkidx[BLOCKIDX_Y0] + 1,
                          gb);
    }

    return 0;
}

static int vc1_decode_p_mb(VC1Context *v,
                           VC1PMBCtx *mbctx,
                           VC1IntraBlkCtx *intra_blkctx,
                           VC1InterBlkCtx *inter_blkctx,
                           int blkidx[BLOCKIDX_MAX],
                           GetBitContext *gb)
{
    VC1StoredBlkCtx *sblkctx = mbctx->s_blkctx;
    VC1StoredBlkCtx *curr_sblkctx = mbctx->s_blkctx + blkidx[BLOCKIDX_Y0];
    unsigned int cbpcy;
    MpegEncContext *s = &v->s;
    VC1PPictCtx *pict = (VC1PPictCtx*)v->pict;
    uint8_t **dest = mbctx->s_mbctx[blkidx[MBIDX]].dest;
    VC1BlkCtx skipped_blkctx = { .btype = BLOCK_SKIPPED };
    int i, j;
    int mquant; /* MB quantization */
    int mb_has_coeffs = 1; /* last_flag */
    int val; /* temp values */
    int dst_idx;
    int skipmbbit, mvmodebit;
    int block_cbp = 0, pat, block_intra = 0;
    int ret;
    int pred_b_blkidx;

    VC1MCCtx luma_mcctx = { .ctype = COMPONENT_TYPE_LUMA,
                            .ref = mbctx->ref[COMPONENT_LUMA],
                            .emulated_edge_mc = s->vdsp.emulated_edge_mc,
                            .edge_emu_buffer = s->sc.edge_emu_buffer,
                            .ic_lut = v->last_luty[0],
                            .stride = mbctx->linesize[COMPONENT_TYPE_LUMA],
                            .replication_edge[MV_X] = s->mb_width * 16,
                            .replication_edge[MV_Y] = s->mb_height * 16,
                            .use_intensity_comp = v->last_use_ic
    };

    VC1MCCtx chroma_mcctx = { .ctype = COMPONENT_TYPE_CHROMA,
                              .emulated_edge_mc = s->vdsp.emulated_edge_mc,
                              .edge_emu_buffer = s->sc.edge_emu_buffer,
                              .ic_lut = v->last_lutuv[0],
                              .stride = mbctx->linesize[COMPONENT_TYPE_CHROMA],
                              .replication_edge[MV_X] = s->mb_width * 8,
                              .replication_edge[MV_Y] = s->mb_height * 8,
                              .block_w = 9,
                              .block_h = 9,
                              .rnd = 8,
                              .use_intensity_comp = v->last_use_ic
    };

    chroma_mcctx.put_pixels_mc_chroma = v->rnd ? v->vc1dsp.put_no_rnd_vc1_chroma_pixels_tab[0] : v->h264chroma.put_h264_chroma_pixels_tab[0];

    // TODO: this changes the crc for SA10164, SA10165 and SA10166
    // However, these streams are Pan and Scan and currently not
    // decoded correctly anyway.
    s->bdsp.clear_blocks(mbctx->block[blkidx[BLOCKIDX_Y0]]);

    mvmodebit = v->mv_type_is_raw ? get_bits1(gb) : v->mv_type_mb_plane[v->s.mb_x + v->s.mb_y * v->s.mb_stride]; // MVMODEBIT
    skipmbbit = v->skip_is_raw ? get_bits1(gb) : v->s.mbskip_table[v->s.mb_x + v->s.mb_y * v->s.mb_stride]; // SKIPMBBIT

    if (mvmodebit) {
        mbctx->mvmode = MV_MODE_MIXED_MV | MV_MODE_BIT;
        luma_mcctx.block_w = 9;
        luma_mcctx.block_h = 9;
        if ((mbctx->mvmode & MV_MODE_MASK) == MV_MODE_1MV_HPEL_BILIN) {
            luma_mcctx.put_pixels_mc_luma = v->rnd ? mbctx->put_pixels_bilin + 5 : mbctx->put_pixels_bilin + 1;
            luma_mcctx.rnd = 8;
        } else {
            luma_mcctx.put_pixels_mc_luma = mbctx->put_pixels_bicubic_8x8;
            luma_mcctx.rnd = v->rnd;
        }
    } else {
        mbctx->mvmode = mbctx->mvmode & ~MV_MODE_BIT;
        luma_mcctx.block_w = 17;
        luma_mcctx.block_h = 17;
        if ((mbctx->mvmode & MV_MODE_MASK) == MV_MODE_1MV_HPEL_BILIN) {
            luma_mcctx.put_pixels_mc_luma = v->rnd ? mbctx->put_pixels_bilin + 4 : mbctx->put_pixels_bilin;
            luma_mcctx.rnd = 16;
        } else {
            luma_mcctx.put_pixels_mc_luma = mbctx->put_pixels_bicubic_16x16;
            luma_mcctx.rnd = v->rnd;
        }
    }

    if (!skipmbbit)
        cbpcy = mvmodebit ? get_vlc2(gb, mbctx->cbpcy_vlc->table, 8, 2) : 1 << 5; // CBPCY
    else
        cbpcy = 0;

    /* block Y0 */
    curr_sblkctx[0].is_coded = cbpcy >> 5 & 1;
    pred_b_blkidx = blkidx[mvmodebit ? BLOCKIDX_LT3 : BLOCKIDX_RT2];
    if (pred_b_blkidx == -1)
        pred_b_blkidx = blkidx[mvmodebit ? BLOCKIDX_T3 : BLOCKIDX_LT3];

    inter_blkctx->blkoffset_qpel[MV_X] = mbctx->mboffset_qpel[MV_X];
    inter_blkctx->blkoffset_qpel[MV_Y] = mbctx->mboffset_qpel[MV_Y];
    luma_mcctx.dest = dest[COMPONENT_LUMA];

    vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0], gb);
    ff_vc1_predict_mv(mbctx,
                      inter_blkctx,
                      blkidx[BLOCKIDX_Y0],
                      blkidx[BLOCKIDX_T2],
                      pred_b_blkidx,
                      blkidx[BLOCKIDX_L1],
                      gb);
    ff_vc1_motion_compensation(&luma_mcctx, inter_blkctx);

    if (mvmodebit) {
        /* block Y1 */
        curr_sblkctx[2].is_coded = cbpcy >> 4 & 1;
        pred_b_blkidx = blkidx[BLOCKIDX_RT2] == -1 ? blkidx[BLOCKIDX_T2] : blkidx[BLOCKIDX_RT2];

        inter_blkctx->blkoffset_qpel[MV_X] += 32;
        luma_mcctx.dest += 8;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 2, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_T3],
                          pred_b_blkidx,
                          blkidx[BLOCKIDX_Y0],
                          gb);
        ff_vc1_motion_compensation(&luma_mcctx, inter_blkctx);

        /* block Y2 */
        curr_sblkctx[1].is_coded = cbpcy >> 3 & 1;

        inter_blkctx->blkoffset_qpel[MV_X] -= 32;
        inter_blkctx->blkoffset_qpel[MV_Y] += 32;
        luma_mcctx.dest += 8 * mbctx->linesize[COMPONENT_TYPE_LUMA] - 8;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 1, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 1,
                          blkidx[BLOCKIDX_Y0],
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_L3],
                          gb);
        ff_vc1_motion_compensation(&luma_mcctx, inter_blkctx);

        /* block Y3 */
        curr_sblkctx[3].is_coded = cbpcy >> 2 & 1;

        inter_blkctx->blkoffset_qpel[MV_X] += 32;
        luma_mcctx.dest += 8;

        vc1_decode_mv_diff(mbctx, blkidx[BLOCKIDX_Y0] + 3, gb);
        ff_vc1_predict_mv(mbctx,
                          inter_blkctx,
                          blkidx[BLOCKIDX_Y0] + 3,
                          blkidx[BLOCKIDX_Y0] + 2,
                          blkidx[BLOCKIDX_Y0],
                          blkidx[BLOCKIDX_Y0] + 1,
                          gb);
        ff_vc1_motion_compensation(&luma_mcctx, inter_blkctx);

        curr_sblkctx[4].is_coded = cbpcy >> 1 & 1;
        curr_sblkctx[5].is_coded = cbpcy & 1;
    } else {
        curr_sblkctx[3].btype =
        curr_sblkctx[2].btype =
        curr_sblkctx[1].btype =
        curr_sblkctx[0].btype;
    }

    inter_blkctx->blkoffset_qpel[MV_X] = mbctx->mboffset_qpel[MV_X] >> 1;
    inter_blkctx->blkoffset_qpel[MV_Y] = mbctx->mboffset_qpel[MV_Y] >> 1;

    ff_vc1_decode_chroma_mv(mbctx, inter_blkctx, blkidx[BLOCKIDX_Y0]);

    /* block Cb */
    chroma_mcctx.dest = dest[COMPONENT_CB];
    chroma_mcctx.ref = mbctx->ref[COMPONENT_CB];

    ff_vc1_motion_compensation(&chroma_mcctx, inter_blkctx);

    /* block Cr */
    chroma_mcctx.dest = dest[COMPONENT_CR];
    chroma_mcctx.ref = mbctx->ref[COMPONENT_CR];

    ff_vc1_motion_compensation(&chroma_mcctx, inter_blkctx);

    if (!skipmbbit) {
        intra_blkctx->use_ac_pred = 0;

        inter_blkctx->tt = mbctx->tt;

        if (!mvmodebit) { /* 1-MV MVMODE */
            if (curr_sblkctx[0].is_coded) {
                if (curr_sblkctx[0].btype == BLOCK_INTRA)
                    intra_blkctx->use_ac_pred = get_bits1(gb); // ACPRED
                cbpcy = get_vlc2(gb, mbctx->cbpcy_vlc->table, 8, 2); // CBPCY
            } else {
                cbpcy = 0;
            }
        }

        for (int i = mvmodebit ? 5 : 0; i >= 0; i--)
            if (curr_sblkctx[i].btype == BLOCK_INTRA || curr_sblkctx[i].is_coded) {
                vc1_decode_mquant(mbctx, gb);
                break;
            }

        if (mvmodebit) { /* 4-MV MVMODE */
            if ((curr_sblkctx[0].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_T2]].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_L1]].btype == BLOCK_INTRA)) ||
                (curr_sblkctx[2].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_T3]].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_Y0]].btype == BLOCK_INTRA)) ||
                (curr_sblkctx[1].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_Y0]].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_L3]].btype == BLOCK_INTRA)) ||
                (curr_sblkctx[3].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_Y0] + 1].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_Y0] + 2].btype == BLOCK_INTRA)) ||
                (curr_sblkctx[4].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_CB_T]].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_CB_L]].btype == BLOCK_INTRA)) ||
                (curr_sblkctx[5].btype == BLOCK_INTRA && (sblkctx[blkidx[BLOCKIDX_CR_T]].btype == BLOCK_INTRA || sblkctx[blkidx[BLOCKIDX_CR_L]].btype == BLOCK_INTRA)))
                intra_blkctx->use_ac_pred = get_bits1(gb); // ACPRED
        } else {
            if (curr_sblkctx[0].btype == BLOCK_INTRA && curr_sblkctx[0].is_coded == 0)
                intra_blkctx->use_ac_pred = get_bits1(gb); // ACPRED
        }

        if (!mbctx->tt) /* MB-level transform type */
            for (int i = mvmodebit ? 5 : 0; i >= 0; i--)
                if (curr_sblkctx[i].btype == BLOCK_INTER && curr_sblkctx[i].is_coded) {
                    inter_blkctx->tt = get_vlc2(gb, mbctx->ttmb_vlc->table, 7, 2); // TTMB
                    break;
                }
    }

    if (!mvmodebit) { /* 1-MV MVMODE */
        curr_sblkctx[0].is_coded = cbpcy >> 5 & 1;
        curr_sblkctx[1].is_coded = cbpcy >> 3 & 1;
        curr_sblkctx[2].is_coded = cbpcy >> 4 & 1;
        curr_sblkctx[3].is_coded = cbpcy >> 2 & 1;
        curr_sblkctx[4].is_coded = cbpcy >> 1 & 1;
        curr_sblkctx[5].is_coded = cbpcy & 1;
    }

    intra_blkctx->mquant = mbctx->mquant;
    intra_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
    intra_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;

    inter_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
    inter_blkctx->ac_level_code_size = &pict->ac_level_code_size;
    inter_blkctx->ac_run_code_size = &pict->ac_run_code_size;
    inter_blkctx->esc_mode3_vlc = pict->pquant < 8 || pict->dqedge;
    inter_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
    inter_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;

    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_LUMA];
    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_LUMA];

    inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];

    if (CONFIG_GRAY) {
        intra_blkctx->skip_output = 0;
        inter_blkctx->skip_output = 0;
    }

    if (!mvmodebit) {
        if (curr_sblkctx[0].btype == BLOCK_INTRA) {
            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                             intra_blkctx,
                                             blkidx[MBIDX],
                                             blkidx[BLOCKIDX_Y0],
                                             blkidx[BLOCKIDX_T2],
                                             blkidx[BLOCKIDX_LT3],
                                             blkidx[BLOCKIDX_L1],
                                             gb);
            if (ret < 0)
                return ret;
        } else {
            inter_blkctx->dest = dest[COMPONENT_LUMA];

            ret = vc1_decode_p_block_new(mbctx,
                                         (VC1BlkCtx*)inter_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0],
                                         blkidx[BLOCKIDX_T2],
                                         blkidx[BLOCKIDX_LT3],
                                         blkidx[BLOCKIDX_L1],
                                         gb);
            if (ret < 0)
                return ret;
        }
    }

    if (!mvmodebit) {
        if (curr_sblkctx[2].btype == BLOCK_INTRA) {
            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                             intra_blkctx,
                                             blkidx[MBIDX],
                                             blkidx[BLOCKIDX_Y0] + 2,
                                             blkidx[BLOCKIDX_T3],
                                             blkidx[BLOCKIDX_T2],
                                             blkidx[BLOCKIDX_Y0],
                                             gb);
            if (ret < 0)
                return ret;
        } else {
            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8;

            ret = vc1_decode_p_block_new(mbctx,
                                         (VC1BlkCtx*)inter_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0] + 2,
                                         blkidx[BLOCKIDX_T3],
                                         blkidx[BLOCKIDX_T2],
                                         blkidx[BLOCKIDX_Y0],
                                         gb);
            if (ret < 0)
                return ret;
        }
    }

    if (!mvmodebit) {
        if (curr_sblkctx[1].btype == BLOCK_INTRA) {
            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                             intra_blkctx,
                                             blkidx[MBIDX],
                                             blkidx[BLOCKIDX_Y0] + 1,
                                             blkidx[BLOCKIDX_Y0],
                                             blkidx[BLOCKIDX_L1],
                                             blkidx[BLOCKIDX_L3],
                                             gb);
            if (ret < 0)
                return ret;
        } else {
            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8;
/*
            ret = vc1_decode_p_block_new(mbctx,
                                         (VC1BlkCtx*)inter_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0] + 1,
                                         blkidx[BLOCKIDX_Y0],
                                         blkidx[BLOCKIDX_L1],
                                         blkidx[BLOCKIDX_L3],
                                         gb);
            if (ret < 0)
                return ret;
*/
        }
    }

    if (!mvmodebit) {
        if (curr_sblkctx[3].btype == BLOCK_INTRA) {
            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                             intra_blkctx,
                                             blkidx[MBIDX],
                                             blkidx[BLOCKIDX_Y0] + 3,
                                             blkidx[BLOCKIDX_Y0] + 2,
                                             blkidx[BLOCKIDX_Y0],
                                             blkidx[BLOCKIDX_Y0] + 1,
                                             gb);
            if (ret < 0)
                return ret;
        } else {
            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8 * mbctx->linesize[COMPONENT_TYPE_LUMA] + 8;
/*
            ret = vc1_decode_p_block_new(mbctx,
                                         (VC1BlkCtx*)inter_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0] + 3,
                                         blkidx[BLOCKIDX_Y0] + 2,
                                         blkidx[BLOCKIDX_Y0],
                                         blkidx[BLOCKIDX_Y0] + 1,
                                         gb);
            if (ret < 0)
                return ret;
*/
        }
    }

    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_CHROMA];
    if (CONFIG_GRAY && mbctx->codec_flag_gray) {
        intra_blkctx->skip_output = 1;
        inter_blkctx->skip_output = 1;
    }

    if (!mvmodebit && curr_sblkctx[4].btype == BLOCK_INTRA) {
        ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                         intra_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0] + 4,
                                         blkidx[BLOCKIDX_CB_T],
                                         blkidx[BLOCKIDX_CB_LT],
                                         blkidx[BLOCKIDX_CB_L],
                                         gb);
        if (ret < 0)
            return ret;
    }

    if (!mvmodebit && curr_sblkctx[5].btype == BLOCK_INTRA) {
        ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                         intra_blkctx,
                                         blkidx[MBIDX],
                                         blkidx[BLOCKIDX_Y0] + 5,
                                         blkidx[BLOCKIDX_CR_T],
                                         blkidx[BLOCKIDX_CR_LT],
                                         blkidx[BLOCKIDX_CR_L],
                                         gb);
        if (ret < 0)
            return ret;
    }

    s->current_picture.motion_val[0][s->block_index[0] + v->blocks_off][0] = curr_sblkctx[0].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[0][s->block_index[0] + v->blocks_off][1] = curr_sblkctx[0].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[0][s->block_index[1] + v->blocks_off][0] = curr_sblkctx[2].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[0][s->block_index[1] + v->blocks_off][1] = curr_sblkctx[2].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[0][s->block_index[2] + v->blocks_off][0] = curr_sblkctx[1].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[0][s->block_index[2] + v->blocks_off][1] = curr_sblkctx[1].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[0][s->block_index[3] + v->blocks_off][0] = curr_sblkctx[3].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[0][s->block_index[3] + v->blocks_off][1] = curr_sblkctx[3].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[1][s->block_index[0] + v->blocks_off][0] = curr_sblkctx[0].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[1][s->block_index[0] + v->blocks_off][1] = curr_sblkctx[0].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[1][s->block_index[1] + v->blocks_off][0] = curr_sblkctx[2].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[1][s->block_index[1] + v->blocks_off][1] = curr_sblkctx[2].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[1][s->block_index[2] + v->blocks_off][0] = curr_sblkctx[1].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[1][s->block_index[2] + v->blocks_off][1] = curr_sblkctx[1].mv[MV_CTX_FORWARD][MV_Y];
    s->current_picture.motion_val[1][s->block_index[3] + v->blocks_off][0] = curr_sblkctx[3].mv[MV_CTX_FORWARD][MV_X];
    s->current_picture.motion_val[1][s->block_index[3] + v->blocks_off][1] = curr_sblkctx[3].mv[MV_CTX_FORWARD][MV_Y];
    v->mb_type[0][s->block_index[0]] = curr_sblkctx[0].btype == BLOCK_INTRA;
    v->mb_type[0][s->block_index[1]] = curr_sblkctx[2].btype == BLOCK_INTRA;
    v->mb_type[0][s->block_index[2]] = curr_sblkctx[1].btype == BLOCK_INTRA;
    v->mb_type[0][s->block_index[3]] = curr_sblkctx[3].btype == BLOCK_INTRA;
    v->mb_type[0][s->block_index[4]] = curr_sblkctx[4].btype == BLOCK_INTRA;
    v->mb_type[0][s->block_index[5]] = curr_sblkctx[5].btype == BLOCK_INTRA;

    mquant = v->pq; /* lossy initialization */

    if (!mvmodebit) { /* 1MV mode */
        if (!skipmbbit) {
            s->mb_intra = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].btype == BLOCK_INTRA;
            mb_has_coeffs = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].is_coded;

            /* FIXME Set DC val for inter block ? */
//            if (s->mb_intra && !mb_has_coeffs) {
//                vc1_decode_mquant(mbctx, gb);
//                s->ac_pred = intra_blkctx->use_ac_pred = get_bits1(gb);
//                cbpcy = 0;
//            } else if (mb_has_coeffs) {
//                if (s->mb_intra)
//                    s->ac_pred = acpred = get_bits1(gb);
//                cbpcy = get_vlc2(&v->s.gb, mbctx->cbpcy_vlc->table, 8, 2); // CBPCY
//                vc1_decode_mquant(mbctx, gb);
//            } else {
//                mbctx->mquant = mbctx->pquant;
//                cbpcy = 0;
//            }

//            inter_blkctx->tt = mbctx->tt;
//            inter_blkctx->ttblk_vlc = mbctx->ttblk_vlc;
//            inter_blkctx->subblkpat_vlc = mbctx->subblkpat_vlc;

//            if (!mbctx->tt && !s->mb_intra && mb_has_coeffs) {
//                inter_blkctx->tt = get_vlc2(gb, mbctx->ttmb_vlc->table, 7, 2); // TTMB
//            }
            dst_idx = 0;
            for (i = 0; i < 6; i++) {
                s->dc_val[0][s->block_index[i]] = 0;
                dst_idx += i >> 2;
                val = ((cbpcy >> (5 - i)) & 1);
//                v->mb_type[0][s->block_index[i]] = s->mb_intra;
                if (s->mb_intra) {
                    /* check if prediction blocks A and C are available */
                    v->a_avail = v->c_avail = 0;
                    if (i == 2 || i == 3 || !s->first_slice_line)
                        v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
                    if (i == 1 || i == 3 || s->mb_x)
                        v->c_avail = v->mb_type[0][s->block_index[i] - 1];

                    mbctx->cbpcy = cbpcy;

                    intra_blkctx->mquant = mbctx->mquant;
                    intra_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
                    intra_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
//                    intra_blkctx->use_ac_pred = acpred;

                    if (v->seq->profile == PROFILE_ADVANCED) {
                        vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, val, mquant,
                                               (i & 4) ? v->codingset2 : v->codingset);
                    } else {
                        /*
                        if (i == 0) {
//                            av_assert0(curr_sblkctx[0].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[1].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[2].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[3].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[4].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[5].btype == BLOCK_INTRA);
//                            av_assert0(curr_sblkctx[0].is_coded == (cbpcy >> 5 & 1));
//                            av_assert0(curr_sblkctx[1].is_coded == (cbpcy >> 3 & 1));
//                            av_assert0(curr_sblkctx[2].is_coded == (cbpcy >> 4 & 1));
//                            av_assert0(curr_sblkctx[3].is_coded == (cbpcy >> 2 & 1));
//                            av_assert0(curr_sblkctx[4].is_coded == (cbpcy >> 1 & 1));
//                            av_assert0(curr_sblkctx[5].is_coded == (cbpcy & 1));
                            ret = vc1_decode_i_mb((VC1IMBCtx*)mbctx,
                                                  intra_blkctx,
                                                  blkidx,
                                                  gb);
                            if (ret < 0)
                                return ret;
                        }
*/
                    }

                    if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                        continue;
                    if (v->rangeredfrm)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[i]][j] *= 2;
                    block_cbp   |= 0xF << (i << 2);
                    block_intra |= 1 << i;
                } else if (val) {
                    if (v->seq->profile == PROFILE_ADVANCED) {
                    } else {
                        inter_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
                        inter_blkctx->ac_level_code_size = &pict->ac_level_code_size;
                        inter_blkctx->ac_run_code_size = &pict->ac_run_code_size;
                        inter_blkctx->esc_mode3_vlc = pict->pquant < 8 || pict->dqedge;
                        inter_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
                        inter_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
                        if (CONFIG_GRAY)
                            inter_blkctx->skip_output = i > 3 && mbctx->codec_flag_gray;

//                        pat = vc1_decode_p_block_new(inter_blkctx, v->block[v->cur_blk_idx][block_map[i]], i,
//                                                     s->dest[dst_idx] + off, (i & 4) ? s->uvlinesize : s->linesize,
//                                                     &block_tt, gb);

                        switch (i) {
                        case 0:
//                            av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[0].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
//                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1],
//                                                         gb);
                            break;

                        case 1:
//                            av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[2].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8;
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
//                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0],
//                                                         gb);
                            break;

                        case 2:
//                            av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[1].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8 * mbctx->linesize[COMPONENT_TYPE_LUMA];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3],
                                                         gb);
                            break;

                        case 3:
//                            av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[3].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8 * mbctx->linesize[COMPONENT_TYPE_LUMA] + 8;
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1,
                                                         gb);
                            break;

                        case 4:
//                            av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[4].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_CB];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_CHROMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L],
                                                         gb);
                            break;

                        case 5:
//                            av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[5].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_CR];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_CHROMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L],
                                                         gb);
                            break;
                        }
                    }
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                } else {
                    switch (i) {
                    case 0:
//                        av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[0].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
//                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1], 0);
                        break;

                    case 1:
//                        av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[2].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
//                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0], 0);
                        break;

                    case 2:
//                        av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[1].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3], 0);
                        break;

                    case 3:
//                        av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[3].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1, 0);
                        break;

                    case 4:
//                        av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[4].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L], 0);
                        break;

                    case 5:
//                        av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[5].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L], 0);
                        break;
                    }
                }
            }
        } else { // skipped
            s->mb_intra = 0;
//            for (i = 0; i < 6; i++) {
//                v->mb_type[0][s->block_index[i]] = 0;
//            }
//            av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[0].is_coded == 0);
//            av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[1].is_coded == 0);
//            av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[2].is_coded == 0);
//            av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[3].is_coded == 0);
//            av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[4].is_coded == 0);
//            av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[5].is_coded == 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
//            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
//            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1, 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L], 0);
        }
    } else { // 4MV mode
        if (!skipmbbit /* unskipped MB */) {
            int intra_count = 0, coded_inter = 0;
            int is_intra[6], is_coded[6];

            for (i = 0; i < 6; i++) {
                val = ((cbpcy >> (5 - i)) & 1);
                s->dc_val[0][s->block_index[i]] = 0;
                s->mb_intra                     = 0;
                if (i < 4) {
                    s->mb_intra   = 0;
                    mb_has_coeffs = 0;
                    if (val) {
                        if (i == 0) {
                            s->mb_intra = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].btype == BLOCK_INTRA;
                            mb_has_coeffs = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].is_coded;
                        }

                        if (i == 1) {
                            s->mb_intra = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].btype == BLOCK_INTRA;
                            mb_has_coeffs = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].is_coded;
                        }

                        if (i == 2) {
                            s->mb_intra = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].btype == BLOCK_INTRA;
                            mb_has_coeffs = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].is_coded;
                        }

                        if (i == 3) {
                            s->mb_intra = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].btype == BLOCK_INTRA;
                            mb_has_coeffs = mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].is_coded;
                        }
                    }
                    intra_count += s->mb_intra;
                    is_intra[i]  = s->mb_intra;
                    is_coded[i]  = mb_has_coeffs;
                }
                if (i & 4) {
                    is_intra[i] = (intra_count >= 3);
                    is_coded[i] = val;
                }
//                v->mb_type[0][s->block_index[i]] = is_intra[i];
                if (!coded_inter)
                    coded_inter = !is_intra[i] & is_coded[i];
            }
            // if there are no coded blocks then don't do anything more
            dst_idx = 0;
            if (!intra_count && !coded_inter){
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1], 0);
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0], 0);
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3], 0);
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1, 0);
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L], 0);
//                mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
                vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L], 0);

                goto end;
            }
//            vc1_decode_mquant(mbctx, gb);
            /* test if block is intra and has pred */
            {
//                int intrapred = 0;
//                for (i = 0; i < 6; i++)
//                    if (is_intra[i]) {
//                        if (((!s->first_slice_line || (i == 2 || i == 3)) && v->mb_type[0][s->block_index[i] - s->block_wrap[i]])
//                            || ((s->mb_x || (i == 1 || i == 3)) && v->mb_type[0][s->block_index[i] - 1])) {
//                            intrapred = 1;
//                            break;
//                        }
//                    }
//                if (intrapred)
//                    s->ac_pred = acpred = get_bits1(gb);
//                else
//                    s->ac_pred = acpred = 0;
            }
            if (v->seq->profile == PROFILE_ADVANCED) {
            } else {
//                inter_blkctx->tt = mbctx->tt;
//                inter_blkctx->ttblk_vlc = mbctx->ttblk_vlc;
//                inter_blkctx->subblkpat_vlc = mbctx->subblkpat_vlc;
//                if (!mbctx->tt && coded_inter) {
//                    inter_blkctx->tt = get_vlc2(gb, mbctx->ttmb_vlc->table, 7, 2); // TTMB
//                }
            }
            for (i = 0; i < 6; i++) {
                dst_idx    += i >> 2;
                s->mb_intra = is_intra[i];
                if (is_intra[i]) {
                    /* check if prediction blocks A and C are available */
                    v->a_avail = v->c_avail = 0;
                    if (i == 2 || i == 3 || !s->first_slice_line)
                        v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
                    if (i == 1 || i == 3 || s->mb_x)
                        v->c_avail = v->mb_type[0][s->block_index[i] - 1];


                    intra_blkctx->mquant = mbctx->mquant;
                    intra_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
                    intra_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
//                    intra_blkctx->use_ac_pred = acpred;

                    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_LUMA];
                    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_LUMA];

                    if (CONFIG_GRAY)
                        intra_blkctx->skip_output = 0;

                    if (i == 0) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y0
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].is_coded = !!is_coded[0];
//                            av_assert0(curr_sblkctx[0].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_T2],
                                                             blkidx[BLOCKIDX_LT3],
                                                             blkidx[BLOCKIDX_L1],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 1) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y1
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].is_coded = !!is_coded[1];
//                            av_assert0(curr_sblkctx[2].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 2,
                                                             blkidx[BLOCKIDX_T3],
                                                             blkidx[BLOCKIDX_T2],
                                                             blkidx[BLOCKIDX_Y0],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 2) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y2
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].is_coded = !!is_coded[2];
//                            av_assert0(curr_sblkctx[1].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 1,
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_L1],
                                                             blkidx[BLOCKIDX_L3],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 3) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y3
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].is_coded = !!is_coded[3];
//                            av_assert0(curr_sblkctx[3].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 3,
                                                             blkidx[BLOCKIDX_Y0] + 2,
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_Y0] + 1,
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

//                    vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
//                                           (i & 4) ? v->codingset2 : v->codingset);
                    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
                    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_CHROMA];

                    if (CONFIG_GRAY && mbctx->codec_flag_gray)
                        intra_blkctx->skip_output = 1;

                    if (i == 4) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Cb
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].is_coded = cbpcy >> 1 & 1;
//                            av_assert0(curr_sblkctx[4].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 4,
                                                             blkidx[BLOCKIDX_CB_T],
                                                             blkidx[BLOCKIDX_CB_LT],
                                                             blkidx[BLOCKIDX_CB_L],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 5) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Cr
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].is_coded = cbpcy & 1;
//                            av_assert0(curr_sblkctx[5].btype == BLOCK_INTRA);
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 5,
                                                             blkidx[BLOCKIDX_CR_T],
                                                             blkidx[BLOCKIDX_CR_LT],
                                                             blkidx[BLOCKIDX_CR_L],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                        continue;

                    if (v->rangeredfrm)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[i]][j] *= 2;
                    block_cbp   |= 0xF << (i << 2);
                    block_intra |= 1 << i;
                } else if (is_coded[i]) {
                    if (v->seq->profile == PROFILE_ADVANCED) {
                    } else {
                        inter_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
                        inter_blkctx->ac_level_code_size = &pict->ac_level_code_size;
                        inter_blkctx->ac_run_code_size = &pict->ac_run_code_size;
                        inter_blkctx->esc_mode3_vlc = pict->pquant < 8 || pict->dqedge;
                        inter_blkctx->double_quant = 2 * mbctx->mquant + (mbctx->mquant_selector == MQUANT_SELECT_PQUANT ? pict->halfqp : 0);
                        inter_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
                        if (CONFIG_GRAY)
                            inter_blkctx->skip_output = i > 3 && mbctx->codec_flag_gray;

//                        pat = vc1_decode_p_block_new(inter_blkctx, v->block[v->cur_blk_idx][block_map[i]], i,
//                                                     s->dest[dst_idx] + off, (i & 4) ? s->uvlinesize : s->linesize,
//                                                     &block_tt, gb);

                        switch (i) {
                        case 0:
//                            av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[0].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1],
                                                         gb);
                            break;

                        case 1:
//                            av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[2].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8;
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0],
                                                         gb);
                            break;

                        case 2:
//                            av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[1].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8 * mbctx->linesize[COMPONENT_TYPE_LUMA];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3],
                                                         gb);
                            break;

                        case 3:
//                            av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[3].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_LUMA] + 8 * mbctx->linesize[COMPONENT_TYPE_LUMA] + 8;
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_LUMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1,
                                                         gb);
                            break;

                        case 4:
//                            av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[4].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_CB];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_CHROMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L],
                                                         gb);
                            break;

                        case 5:
//                            av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//                            av_assert0(curr_sblkctx[5].is_coded);
                            inter_blkctx->dest = dest[COMPONENT_CR];
                            inter_blkctx->linesize = mbctx->linesize[COMPONENT_TYPE_CHROMA];
//                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
                            pat = vc1_decode_p_block_new(mbctx, (VC1BlkCtx*)inter_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L],
                                                         gb);
                            break;
                        }
                    }
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                } else {
                    switch (i) {
                    case 0:
//                        av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[0].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1], 0);
                        break;

                    case 1:
//                        av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[2].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0], 0);
                        break;

                    case 2:
//                        av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[1].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3], 0);
                        break;

                    case 3:
//                        av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[3].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1, 0);
                        break;

                    case 4:
//                        av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[4].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L], 0);
                        break;

                    case 5:
//                        av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//                        av_assert0(curr_sblkctx[5].is_coded == 0);
//                        mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
                        vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L], 0);
                        break;
                    }
                }
            }
        } else { // skipped MB
            s->mb_intra                               = 0;
//            for (i = 0; i < 6; i++) {
//                v->mb_type[0][s->block_index[i]] = 0;
//            }
//            av_assert0(curr_sblkctx[0].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[0].is_coded == 0);
//            av_assert0(curr_sblkctx[1].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[1].is_coded == 0);
//            av_assert0(curr_sblkctx[2].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[2].is_coded == 0);
//            av_assert0(curr_sblkctx[3].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[3].is_coded == 0);
//            av_assert0(curr_sblkctx[4].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[4].is_coded == 0);
//            av_assert0(curr_sblkctx[5].btype == BLOCK_INTER);
//            av_assert0(curr_sblkctx[5].is_coded == 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_LT3], blkidx[BLOCKIDX_L1], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_T3], blkidx[BLOCKIDX_T2], blkidx[BLOCKIDX_Y0], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 1, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_L1], blkidx[BLOCKIDX_L3], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 3, blkidx[BLOCKIDX_Y0] + 2, blkidx[BLOCKIDX_Y0], blkidx[BLOCKIDX_Y0] + 1, 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 4, blkidx[BLOCKIDX_CB_T], blkidx[BLOCKIDX_CB_LT], blkidx[BLOCKIDX_CB_L], 0);
//            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].loopfilter = LOOPFILTER_NONE;
            vc1_decode_p_block_new(mbctx, &skipped_blkctx, blkidx[MBIDX], blkidx[BLOCKIDX_Y0] + 5, blkidx[BLOCKIDX_CR_T], blkidx[BLOCKIDX_CR_LT], blkidx[BLOCKIDX_CR_L], 0);
        }
    }
end:
    v->cbp[s->mb_x]      = block_cbp;
    v->is_intra[s->mb_x] = block_intra;

    return 0;
}

static int vc1_decode_p_mb_adv(VC1Context *v,
                               VC1PMBCtx *mbctx,
                               VC1IntraBlkCtx *intra_blkctx,
                               VC1InterBlkCtx *inter_blkctx,
                               int blkidx[BLOCKIDX_MAX],
                               GetBitContext *gb)
{
    MpegEncContext *s = &v->s;
    VC1PPictCtx *pict = (VC1PPictCtx*)v->pict;
    int acpred, cbpcy;
    int i, j;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */

    int mb_has_coeffs = 1; /* last_flag */
    int dmv_x, dmv_y; /* Differential MV components */
    int index, index1; /* LUT indexes */
    int val, sign; /* temp values */
    int first_block = 1;
    int dst_idx, off;
    int skipped, fourmv;
    int block_cbp = 0, pat, block_tt = 0, block_intra = 0;
    int ret;

    mquant = v->pq; /* lossy initialization */

    // TODO: this changes the crc for SA10164, SA10165 and SA10166
    // However, these streams are Pan and Scan and currently not
    // decoded correctly anyway.
    s->bdsp.clear_blocks(mbctx->block[blkidx[BLOCKIDX_Y0]]);

    if (v->mv_type_is_raw)
        fourmv = get_bits1(gb);
    else
        fourmv = v->mv_type_mb_plane[mb_pos];
    if (v->skip_is_raw)
        skipped = get_bits1(gb);
    else
        skipped = v->s.mbskip_table[mb_pos];

    if (!fourmv) { /* 1MV mode */
        if (!skipped) {
            GET_MVDATA(dmv_x, dmv_y);

            if (s->mb_intra) {
                s->current_picture.motion_val[1][s->block_index[0]][0] = 0;
                s->current_picture.motion_val[1][s->block_index[0]][1] = 0;
            }
            s->current_picture.mb_type[mb_pos] = s->mb_intra ? MB_TYPE_INTRA : MB_TYPE_16x16;
            ff_vc1_pred_mv(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], 0, 0);

            /* FIXME Set DC val for inter block ? */
            if (s->mb_intra && !mb_has_coeffs) {
                GET_MQUANT();
                s->ac_pred = acpred = get_bits1(gb);
                cbp = cbpcy = 0;
            } else if (mb_has_coeffs) {
                if (s->mb_intra)
                    s->ac_pred = acpred = get_bits1(gb);
                if (v->seq->profile == PROFILE_ADVANCED)
                    cbp = cbpcy = get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
                else
                    cbp = cbpcy = get_vlc2(&v->s.gb, mbctx->cbpcy_vlc->table, 8, 2); // CBPCY
                GET_MQUANT();
            } else {
                mquant = v->pq;
                cbp = cbpcy = 0;
            }
            s->current_picture.qscale_table[mb_pos] = mquant;

            if (v->seq->profile == PROFILE_ADVANCED) {
                if (!v->ttmbf && !s->mb_intra && mb_has_coeffs)
                    ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table,
                                    VC1_TTMB_VLC_BITS, 2);
            } else {
                inter_blkctx->tt = mbctx->tt;
                inter_blkctx->ttblk_vlc = mbctx->ttblk_vlc;
                inter_blkctx->subblkpat_vlc = mbctx->subblkpat_vlc;

                if (!mbctx->tt && !s->mb_intra && mb_has_coeffs) {
                    int index;

                    index= gb->index;
                    ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table,
                                    VC1_TTMB_VLC_BITS, 2);

                    gb->index = index;
                    inter_blkctx->tt = get_vlc2(gb, mbctx->ttmb_vlc->table, 7, 2); // TTMB
                }
            }
            if (!s->mb_intra) ff_vc1_mc_1mv(v, 0);
            dst_idx = 0;
            for (i = 0; i < 6; i++) {
                s->dc_val[0][s->block_index[i]] = 0;
                dst_idx += i >> 2;
                val = ((cbp >> (5 - i)) & 1);
                off = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
                v->mb_type[0][s->block_index[i]] = s->mb_intra;
                if (s->mb_intra) {
                    /* check if prediction blocks A and C are available */
                    v->a_avail = v->c_avail = 0;
                    if (i == 2 || i == 3 || !s->first_slice_line)
                        v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
                    if (i == 1 || i == 3 || s->mb_x)
                        v->c_avail = v->mb_type[0][s->block_index[i] - 1];

                    mbctx->mquant = FFABS(mquant);
                    mbctx->cbpcy = cbpcy;

                    intra_blkctx->mquant = mbctx->mquant;
                    intra_blkctx->double_quant = 2 * mbctx->mquant + pict->halfqp * (mquant > 0);
                    intra_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
                    intra_blkctx->use_ac_pred = acpred;

                    if (v->seq->profile == PROFILE_ADVANCED) {
                        vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, val, mquant,
                                               (i & 4) ? v->codingset2 : v->codingset);
                    } else {
                        if (i == 0) {
                            ret = vc1_decode_i_mb((VC1IMBCtx*)mbctx,
                                                  intra_blkctx,
                                                  blkidx,
                                                  gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                        continue;
                    if (v->seq->profile == PROFILE_ADVANCED)
                        v->vc1dsp.vc1_inv_trans_8x8(v->block[v->cur_blk_idx][block_map[i]]);
                    if (v->rangeredfrm)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[i]][j] *= 2;
                    block_cbp   |= 0xF << (i << 2);
                    block_intra |= 1 << i;
                } else if (val) {
                    if (v->seq->profile == PROFILE_ADVANCED) {
                        pat = vc1_decode_p_block(v, v->block[v->cur_blk_idx][block_map[i]], i, mquant, ttmb, first_block,
                                                 s->dest[dst_idx] + off, (i & 4) ? s->uvlinesize : s->linesize,
                                                 CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY), &block_tt);
                    }
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                    if (!v->ttmbf && ttmb < 8)
                        ttmb = -1;
                    first_block = 0;
                }
            }
        } else { // skipped
            s->mb_intra = 0;
            for (i = 0; i < 6; i++) {
                v->mb_type[0][s->block_index[i]] = 0;
                s->dc_val[0][s->block_index[i]]  = 0;
            }
            s->current_picture.mb_type[mb_pos]      = MB_TYPE_SKIP;
            s->current_picture.qscale_table[mb_pos] = 0;
            ff_vc1_pred_mv(v, 0, 0, 0, 1, v->range_x, v->range_y, v->mb_type[0], 0, 0);
            ff_vc1_mc_1mv(v, 0);
        }
    } else { // 4MV mode
        if (!skipped /* unskipped MB */) {
            int intra_count = 0, coded_inter = 0;
            int is_intra[6], is_coded[6];
            /* Get CBPCY */
            if (v->seq->profile == PROFILE_ADVANCED)
                cbp = get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
            else
                cbp = cbpcy = get_vlc2(&v->s.gb, mbctx->cbpcy_vlc->table, 8, 2); // CBPCY
            for (i = 0; i < 6; i++) {
                val = ((cbp >> (5 - i)) & 1);
                s->dc_val[0][s->block_index[i]] = 0;
                s->mb_intra                     = 0;
                if (i < 4) {
                    dmv_x = dmv_y = 0;
                    s->mb_intra   = 0;
                    mb_has_coeffs = 0;
                    if (val) {
                        GET_MVDATA(dmv_x, dmv_y);
                    }
                    ff_vc1_pred_mv(v, i, dmv_x, dmv_y, 0, v->range_x, v->range_y, v->mb_type[0], 0, 0);
                    if (!s->mb_intra)
                        ff_vc1_mc_4mv_luma(v, i, 0, 0);
                    intra_count += s->mb_intra;
                    is_intra[i]  = s->mb_intra;
                    is_coded[i]  = mb_has_coeffs;
                }
                if (i & 4) {
                    is_intra[i] = (intra_count >= 3);
                    is_coded[i] = val;
                }
                if (i == 4)
                    ff_vc1_mc_4mv_chroma(v, 0);
                v->mb_type[0][s->block_index[i]] = is_intra[i];
                if (!coded_inter)
                    coded_inter = !is_intra[i] & is_coded[i];
            }
            // if there are no coded blocks then don't do anything more
            dst_idx = 0;
            if (!intra_count && !coded_inter){
                goto end;
            }
            GET_MQUANT();
            s->current_picture.qscale_table[mb_pos] = mquant;
            /* test if block is intra and has pred */
            {
                int intrapred = 0;
                for (i = 0; i < 6; i++)
                    if (is_intra[i]) {
                        if (((!s->first_slice_line || (i == 2 || i == 3)) && v->mb_type[0][s->block_index[i] - s->block_wrap[i]])
                            || ((s->mb_x || (i == 1 || i == 3)) && v->mb_type[0][s->block_index[i] - 1])) {
                            intrapred = 1;
                            break;
                        }
                    }
                if (intrapred)
                    s->ac_pred = acpred = get_bits1(gb);
                else
                    s->ac_pred = acpred = 0;
            }
            if (v->seq->profile == PROFILE_ADVANCED) {
                if (!v->ttmbf && coded_inter)
                    ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
            } else {
                inter_blkctx->tt = mbctx->tt;
                inter_blkctx->ttblk_vlc = mbctx->ttblk_vlc;
                inter_blkctx->subblkpat_vlc = mbctx->subblkpat_vlc;
                if (!mbctx->tt && coded_inter) {
                    int index;

                    index= gb->index;
                    ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table,
                                    VC1_TTMB_VLC_BITS, 2);

                    gb->index = index;
                    inter_blkctx->tt = get_vlc2(gb, mbctx->ttmb_vlc->table, 7, 2); // TTMB
                }
            }
            for (i = 0; i < 6; i++) {
                dst_idx    += i >> 2;
                off         = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
                s->mb_intra = is_intra[i];
                if (is_intra[i]) {
                    /* check if prediction blocks A and C are available */
                    v->a_avail = v->c_avail = 0;
                    if (i == 2 || i == 3 || !s->first_slice_line)
                        v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
                    if (i == 1 || i == 3 || s->mb_x)
                        v->c_avail = v->mb_type[0][s->block_index[i] - 1];

                    mbctx->mquant = FFABS(mquant);

                    intra_blkctx->mquant = mbctx->mquant;
                    intra_blkctx->double_quant = 2 * mbctx->mquant + pict->halfqp * (mquant > 0);
                    intra_blkctx->quant_scale = mbctx->mquant * !pict->pquantizer;
                    intra_blkctx->use_ac_pred = acpred;

                    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_LUMA];
                    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_LUMA];

                    if (CONFIG_GRAY)
                        intra_blkctx->skip_output = 0;

                    if (i == 0) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y0
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0]].is_coded = !!is_coded[0];
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_T2],
                                                             blkidx[BLOCKIDX_LT3],
                                                             blkidx[BLOCKIDX_L1],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 1) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y1
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 2].is_coded = !!is_coded[1];
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 2,
                                                             blkidx[BLOCKIDX_T3],
                                                             blkidx[BLOCKIDX_T2],
                                                             blkidx[BLOCKIDX_Y0],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 2) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y2
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 1].is_coded = !!is_coded[2];
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 1,
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_L1],
                                                             blkidx[BLOCKIDX_L3],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 3) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Y3
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 3].is_coded = !!is_coded[3];
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 3,
                                                             blkidx[BLOCKIDX_Y0] + 2,
                                                             blkidx[BLOCKIDX_Y0],
                                                             blkidx[BLOCKIDX_Y0] + 1,
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

//                    vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
//                                           (i & 4) ? v->codingset2 : v->codingset);
                    intra_blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
                    intra_blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_CHROMA];

                    if (CONFIG_GRAY && mbctx->codec_flag_gray)
                        intra_blkctx->skip_output = 1;

                    if (i == 4) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Cb
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 4].is_coded = cbpcy >> 1 & 1;
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 4,
                                                             blkidx[BLOCKIDX_CB_T],
                                                             blkidx[BLOCKIDX_CB_LT],
                                                             blkidx[BLOCKIDX_CB_L],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (i == 5) {
                        if (v->seq->profile == PROFILE_ADVANCED) {
                            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, is_coded[i], mquant,
                                                   (i & 4) ? v->codingset2 : v->codingset);
                        } else {
                            // decode block Cr
                            mbctx->s_blkctx[blkidx[BLOCKIDX_Y0] + 5].is_coded = cbpcy & 1;
                            ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                                             intra_blkctx,
                                                             blkidx[MBIDX],
                                                             blkidx[BLOCKIDX_Y0] + 5,
                                                             blkidx[BLOCKIDX_CR_T],
                                                             blkidx[BLOCKIDX_CR_LT],
                                                             blkidx[BLOCKIDX_CR_L],
                                                             gb);
                            if (ret < 0)
                                return ret;
                        }
                    }

                    if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                        continue;

                    if (v->seq->profile == PROFILE_ADVANCED)
                        v->vc1dsp.vc1_inv_trans_8x8(v->block[v->cur_blk_idx][block_map[i]]);
                    if (v->rangeredfrm)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[i]][j] *= 2;
                    block_cbp   |= 0xF << (i << 2);
                    block_intra |= 1 << i;
                } else if (is_coded[i]) {
                    if (v->seq->profile == PROFILE_ADVANCED) {
                        pat = vc1_decode_p_block(v, v->block[v->cur_blk_idx][block_map[i]], i, mquant, ttmb,
                                                 first_block, s->dest[dst_idx] + off,
                                                 (i & 4) ? s->uvlinesize : s->linesize,
                                                 CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY),
                                                 &block_tt);
                    }
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                    if (!v->ttmbf && ttmb < 8)
                        ttmb = -1;
                    first_block = 0;
                }
            }
        } else { // skipped MB
            s->mb_intra                               = 0;
            s->current_picture.qscale_table[mb_pos] = 0;
            for (i = 0; i < 6; i++) {
                v->mb_type[0][s->block_index[i]] = 0;
                s->dc_val[0][s->block_index[i]]  = 0;
            }
            for (i = 0; i < 4; i++) {
                ff_vc1_pred_mv(v, i, 0, 0, 0, v->range_x, v->range_y, v->mb_type[0], 0, 0);
                ff_vc1_mc_4mv_luma(v, i, 0, 0);
            }
            ff_vc1_mc_4mv_chroma(v, 0);
            s->current_picture.qscale_table[mb_pos] = 0;
        }
    }
end:
    if (v->seq->profile == PROFILE_ADVANCED) {
        if (v->overlap && v->pq >= 9)
            ff_vc1_p_overlap_filter(v);
        vc1_put_blocks_clamped(v, 1);
    }

    v->cbp[s->mb_x]      = block_cbp;
    v->ttblk[s->mb_x]    = block_tt;
    v->is_intra[s->mb_x] = block_intra;

    return 0;
}

/* Decode one macroblock in an interlaced frame p picture */

static int vc1_decode_p_mb_intfr(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp = 0; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */

    int mb_has_coeffs = 1; /* last_flag */
    int dmv_x, dmv_y; /* Differential MV components */
    int val; /* temp value */
    int first_block = 1;
    int dst_idx, off;
    int skipped, fourmv = 0, twomv = 0;
    int block_cbp = 0, pat, block_tt = 0;
    int idx_mbmode = 0, mvbp;
    int fieldtx;

    mquant = v->pq; /* Lossy initialization */

    if (v->skip_is_raw)
        skipped = get_bits1(gb);
    else
        skipped = v->s.mbskip_table[mb_pos];
    if (!skipped) {
        if (v->fourmvswitch)
            idx_mbmode = get_vlc2(gb, v->mbmode_vlc->table, VC1_INTFR_4MV_MBMODE_VLC_BITS, 2); // try getting this done
        else
            idx_mbmode = get_vlc2(gb, v->mbmode_vlc->table, VC1_INTFR_NON4MV_MBMODE_VLC_BITS, 2); // in a single line
        switch (ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][0]) {
        /* store the motion vector type in a flag (useful later) */
        case MV_PMODE_INTFR_4MV:
            fourmv = 1;
            v->blk_mv_type[s->block_index[0]] = 0;
            v->blk_mv_type[s->block_index[1]] = 0;
            v->blk_mv_type[s->block_index[2]] = 0;
            v->blk_mv_type[s->block_index[3]] = 0;
            break;
        case MV_PMODE_INTFR_4MV_FIELD:
            fourmv = 1;
            v->blk_mv_type[s->block_index[0]] = 1;
            v->blk_mv_type[s->block_index[1]] = 1;
            v->blk_mv_type[s->block_index[2]] = 1;
            v->blk_mv_type[s->block_index[3]] = 1;
            break;
        case MV_PMODE_INTFR_2MV_FIELD:
            twomv = 1;
            v->blk_mv_type[s->block_index[0]] = 1;
            v->blk_mv_type[s->block_index[1]] = 1;
            v->blk_mv_type[s->block_index[2]] = 1;
            v->blk_mv_type[s->block_index[3]] = 1;
            break;
        case MV_PMODE_INTFR_1MV:
            v->blk_mv_type[s->block_index[0]] = 0;
            v->blk_mv_type[s->block_index[1]] = 0;
            v->blk_mv_type[s->block_index[2]] = 0;
            v->blk_mv_type[s->block_index[3]] = 0;
            break;
        }
        if (ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][0] == MV_PMODE_INTFR_INTRA) { // intra MB
            for (i = 0; i < 4; i++) {
                s->current_picture.motion_val[1][s->block_index[i]][0] = 0;
                s->current_picture.motion_val[1][s->block_index[i]][1] = 0;
            }
            v->is_intra[s->mb_x] = 0x3f; // Set the bitfield to all 1.
            s->mb_intra          = 1;
            s->current_picture.mb_type[mb_pos] = MB_TYPE_INTRA;
            fieldtx = v->fieldtx_plane[mb_pos] = get_bits1(gb);
            mb_has_coeffs = get_bits1(gb);
            if (mb_has_coeffs)
                cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
            v->s.ac_pred = v->acpred_plane[mb_pos] = get_bits1(gb);
            GET_MQUANT();
            s->current_picture.qscale_table[mb_pos] = mquant;
            /* Set DC scale - y and c use the same (not sure if necessary here) */
            s->y_dc_scale = s->y_dc_scale_table[FFABS(mquant)];
            s->c_dc_scale = s->c_dc_scale_table[FFABS(mquant)];
            dst_idx = 0;
            for (i = 0; i < 6; i++) {
                v->a_avail = v->c_avail          = 0;
                v->mb_type[0][s->block_index[i]] = 1;
                s->dc_val[0][s->block_index[i]]  = 0;
                dst_idx += i >> 2;
                val = ((cbp >> (5 - i)) & 1);
                if (i == 2 || i == 3 || !s->first_slice_line)
                    v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
                if (i == 1 || i == 3 || s->mb_x)
                    v->c_avail = v->mb_type[0][s->block_index[i] - 1];

                vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, val, mquant,
                                       (i & 4) ? v->codingset2 : v->codingset);
                if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                    continue;
                v->vc1dsp.vc1_inv_trans_8x8(v->block[v->cur_blk_idx][block_map[i]]);
                if (i < 4)
                    off = (fieldtx) ? ((i & 1) * 8) + ((i & 2) >> 1) * s->linesize : (i & 1) * 8 + 4 * (i & 2) * s->linesize;
                else
                    off = 0;
                block_cbp |= 0xf << (i << 2);
            }

        } else { // inter MB
            mb_has_coeffs = ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][3];
            if (mb_has_coeffs)
                cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
            if (ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][0] == MV_PMODE_INTFR_2MV_FIELD) {
                v->twomvbp = get_vlc2(gb, v->twomvbp_vlc->table, VC1_2MV_BLOCK_PATTERN_VLC_BITS, 1);
            } else {
                if ((ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][0] == MV_PMODE_INTFR_4MV)
                    || (ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][0] == MV_PMODE_INTFR_4MV_FIELD)) {
                    v->fourmvbp = get_vlc2(gb, v->fourmvbp_vlc->table, VC1_4MV_BLOCK_PATTERN_VLC_BITS, 1);
                }
            }
            s->mb_intra = v->is_intra[s->mb_x] = 0;
            for (i = 0; i < 6; i++)
                v->mb_type[0][s->block_index[i]] = 0;
            fieldtx = v->fieldtx_plane[mb_pos] = ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][1];
            /* for all motion vector read MVDATA and motion compensate each block */
            dst_idx = 0;
            if (fourmv) {
                mvbp = v->fourmvbp;
                for (i = 0; i < 4; i++) {
                    dmv_x = dmv_y = 0;
                    if (mvbp & (8 >> i))
                        get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                    ff_vc1_pred_mv_intfr(v, i, dmv_x, dmv_y, 0, v->range_x, v->range_y, v->mb_type[0], 0);
                    ff_vc1_mc_4mv_luma(v, i, 0, 0);
                }
                ff_vc1_mc_4mv_chroma4(v, 0, 0, 0);
            } else if (twomv) {
                mvbp  = v->twomvbp;
                dmv_x = dmv_y = 0;
                if (mvbp & 2) {
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                }
                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 2, v->range_x, v->range_y, v->mb_type[0], 0);
                ff_vc1_mc_4mv_luma(v, 0, 0, 0);
                ff_vc1_mc_4mv_luma(v, 1, 0, 0);
                dmv_x = dmv_y = 0;
                if (mvbp & 1) {
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                }
                ff_vc1_pred_mv_intfr(v, 2, dmv_x, dmv_y, 2, v->range_x, v->range_y, v->mb_type[0], 0);
                ff_vc1_mc_4mv_luma(v, 2, 0, 0);
                ff_vc1_mc_4mv_luma(v, 3, 0, 0);
                ff_vc1_mc_4mv_chroma4(v, 0, 0, 0);
            } else {
                mvbp = ff_vc1_mbmode_intfrp[v->fourmvswitch][idx_mbmode][2];
                dmv_x = dmv_y = 0;
                if (mvbp) {
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                }
                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], 0);
                ff_vc1_mc_1mv(v, 0);
            }
            if (cbp)
                GET_MQUANT();  // p. 227
            s->current_picture.qscale_table[mb_pos] = mquant;
            if (!v->ttmbf && cbp)
                ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
            for (i = 0; i < 6; i++) {
                s->dc_val[0][s->block_index[i]] = 0;
                dst_idx += i >> 2;
                val = ((cbp >> (5 - i)) & 1);
                if (!fieldtx)
                    off = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
                else
                    off = (i & 4) ? 0 : ((i & 1) * 8 + ((i > 1) * s->linesize));
                if (val) {
                    pat = vc1_decode_p_block(v, v->block[v->cur_blk_idx][block_map[i]], i, mquant, ttmb,
                                             first_block, s->dest[dst_idx] + off,
                                             (i & 4) ? s->uvlinesize : (s->linesize << fieldtx),
                                             CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY), &block_tt);
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                    if (!v->ttmbf && ttmb < 8)
                        ttmb = -1;
                    first_block = 0;
                }
            }
        }
    } else { // skipped
        s->mb_intra = v->is_intra[s->mb_x] = 0;
        for (i = 0; i < 6; i++) {
            v->mb_type[0][s->block_index[i]] = 0;
            s->dc_val[0][s->block_index[i]] = 0;
        }
        s->current_picture.mb_type[mb_pos]      = MB_TYPE_SKIP;
        s->current_picture.qscale_table[mb_pos] = 0;
        v->blk_mv_type[s->block_index[0]] = 0;
        v->blk_mv_type[s->block_index[1]] = 0;
        v->blk_mv_type[s->block_index[2]] = 0;
        v->blk_mv_type[s->block_index[3]] = 0;
        ff_vc1_pred_mv_intfr(v, 0, 0, 0, 1, v->range_x, v->range_y, v->mb_type[0], 0);
        ff_vc1_mc_1mv(v, 0);
        v->fieldtx_plane[mb_pos] = 0;
    }
    if (v->overlap && v->pq >= 9)
        ff_vc1_p_overlap_filter(v);
    vc1_put_blocks_clamped(v, 1);

    v->cbp[s->mb_x]      = block_cbp;
    v->ttblk[s->mb_x]    = block_tt;

    return 0;
}

static int vc1_decode_p_mb_intfi(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp = 0; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */

    int mb_has_coeffs = 1; /* last_flag */
    int dmv_x, dmv_y; /* Differential MV components */
    int val; /* temp values */
    int first_block = 1;
    int dst_idx, off;
    int pred_flag = 0;
    int block_cbp = 0, pat, block_tt = 0;
    int idx_mbmode = 0;

    mquant = v->pq; /* Lossy initialization */

    idx_mbmode = get_vlc2(gb, v->mbmode_vlc->table, VC1_IF_MBMODE_VLC_BITS, 2);
    if (idx_mbmode <= 1) { // intra MB
        v->is_intra[s->mb_x] = 0x3f; // Set the bitfield to all 1.
        s->mb_intra          = 1;
        s->current_picture.motion_val[1][s->block_index[0] + v->blocks_off][0] = 0;
        s->current_picture.motion_val[1][s->block_index[0] + v->blocks_off][1] = 0;
        s->current_picture.mb_type[mb_pos + v->mb_off] = MB_TYPE_INTRA;
        GET_MQUANT();
        s->current_picture.qscale_table[mb_pos] = mquant;
        /* Set DC scale - y and c use the same (not sure if necessary here) */
        s->y_dc_scale = s->y_dc_scale_table[FFABS(mquant)];
        s->c_dc_scale = s->c_dc_scale_table[FFABS(mquant)];
        v->s.ac_pred  = v->acpred_plane[mb_pos] = get_bits1(gb);
        mb_has_coeffs = idx_mbmode & 1;
        if (mb_has_coeffs)
            cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_ICBPCY_VLC_BITS, 2);
        dst_idx = 0;
        for (i = 0; i < 6; i++) {
            v->a_avail = v->c_avail          = 0;
            v->mb_type[0][s->block_index[i]] = 1;
            s->dc_val[0][s->block_index[i]]  = 0;
            dst_idx += i >> 2;
            val = ((cbp >> (5 - i)) & 1);
            if (i == 2 || i == 3 || !s->first_slice_line)
                v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
            if (i == 1 || i == 3 || s->mb_x)
                v->c_avail = v->mb_type[0][s->block_index[i] - 1];

            vc1_decode_intra_block(v, v->block[v->cur_blk_idx][block_map[i]], i, val, mquant,
                                   (i & 4) ? v->codingset2 : v->codingset);
            if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                continue;
            v->vc1dsp.vc1_inv_trans_8x8(v->block[v->cur_blk_idx][block_map[i]]);
            off  = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
            block_cbp |= 0xf << (i << 2);
        }
    } else {
        s->mb_intra = v->is_intra[s->mb_x] = 0;
        s->current_picture.mb_type[mb_pos + v->mb_off] = MB_TYPE_16x16;
        for (i = 0; i < 6; i++)
            v->mb_type[0][s->block_index[i]] = 0;
        if (idx_mbmode <= 5) { // 1-MV
            dmv_x = dmv_y = pred_flag = 0;
            if (idx_mbmode & 1) {
                get_mvdata_interlaced(v, &dmv_x, &dmv_y, &pred_flag);
            }
            ff_vc1_pred_mv(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], pred_flag, 0);
            ff_vc1_mc_1mv(v, 0);
            mb_has_coeffs = !(idx_mbmode & 2);
        } else { // 4-MV
            v->fourmvbp = get_vlc2(gb, v->fourmvbp_vlc->table, VC1_4MV_BLOCK_PATTERN_VLC_BITS, 1);
            for (i = 0; i < 4; i++) {
                dmv_x = dmv_y = pred_flag = 0;
                if (v->fourmvbp & (8 >> i))
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, &pred_flag);
                ff_vc1_pred_mv(v, i, dmv_x, dmv_y, 0, v->range_x, v->range_y, v->mb_type[0], pred_flag, 0);
                ff_vc1_mc_4mv_luma(v, i, 0, 0);
            }
            ff_vc1_mc_4mv_chroma(v, 0);
            mb_has_coeffs = idx_mbmode & 1;
        }
        if (mb_has_coeffs)
            cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
        if (cbp) {
            GET_MQUANT();
        }
        s->current_picture.qscale_table[mb_pos] = mquant;
        if (!v->ttmbf && cbp) {
            ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
        }
        dst_idx = 0;
        for (i = 0; i < 6; i++) {
            s->dc_val[0][s->block_index[i]] = 0;
            dst_idx += i >> 2;
            val = ((cbp >> (5 - i)) & 1);
            off = (i & 4) ? 0 : (i & 1) * 8 + (i & 2) * 4 * s->linesize;
            if (val) {
                pat = vc1_decode_p_block(v, v->block[v->cur_blk_idx][block_map[i]], i, mquant, ttmb,
                                         first_block, s->dest[dst_idx] + off,
                                         (i & 4) ? s->uvlinesize : s->linesize,
                                         CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY),
                                         &block_tt);
                if (pat < 0)
                    return pat;
                block_cbp |= pat << (i << 2);
                if (!v->ttmbf && ttmb < 8)
                    ttmb = -1;
                first_block = 0;
            }
        }
    }
    if (v->overlap && v->pq >= 9)
        ff_vc1_p_overlap_filter(v);
    vc1_put_blocks_clamped(v, 1);

    v->cbp[s->mb_x]      = block_cbp;
    v->ttblk[s->mb_x]    = block_tt;

    return 0;
}

/** Decode one B-frame MB (in Main profile)
 */
static int vc1_decode_b_mb(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i, j;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp = 0; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */
    int mb_has_coeffs = 0; /* last_flag */
    int index, index1; /* LUT indexes */
    int val, sign; /* temp values */
    int first_block = 1;
    int dst_idx, off;
    int skipped, direct;
    int dmv_x[2], dmv_y[2];
    int bmvtype = BMV_TYPE_BACKWARD;

    mquant      = v->pq; /* lossy initialization */
    s->mb_intra = 0;

    if (v->dmb_is_raw)
        direct = get_bits1(gb);
    else
        direct = v->direct_mb_plane[mb_pos];
    if (v->skip_is_raw)
        skipped = get_bits1(gb);
    else
        skipped = v->s.mbskip_table[mb_pos];

    dmv_x[0] = dmv_x[1] = dmv_y[0] = dmv_y[1] = 0;
    for (i = 0; i < 6; i++) {
        v->mb_type[0][s->block_index[i]] = 0;
        s->dc_val[0][s->block_index[i]]  = 0;
    }
    s->current_picture.qscale_table[mb_pos] = 0;

    if (!direct) {
        if (!skipped) {
            GET_MVDATA(dmv_x[0], dmv_y[0]);
            dmv_x[1] = dmv_x[0];
            dmv_y[1] = dmv_y[0];
        }
        if (skipped || !s->mb_intra) {
            bmvtype = decode012(gb);
            switch (bmvtype) {
            case 0:
                bmvtype = (v->bfraction >= (B_FRACTION_DEN/2)) ? BMV_TYPE_BACKWARD : BMV_TYPE_FORWARD;
                break;
            case 1:
                bmvtype = (v->bfraction >= (B_FRACTION_DEN/2)) ? BMV_TYPE_FORWARD : BMV_TYPE_BACKWARD;
                break;
            case 2:
                bmvtype  = BMV_TYPE_INTERPOLATED;
                dmv_x[0] = dmv_y[0] = 0;
            }
        }
    }
    for (i = 0; i < 6; i++)
        v->mb_type[0][s->block_index[i]] = s->mb_intra;

    if (skipped) {
        if (direct)
            bmvtype = BMV_TYPE_INTERPOLATED;
        ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
        vc1_b_mc(v, dmv_x, dmv_y, direct, bmvtype);
        return 0;
    }
    if (direct) {
        cbp = get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
        GET_MQUANT();
        s->mb_intra = 0;
        s->current_picture.qscale_table[mb_pos] = mquant;
        if (!v->ttmbf)
            ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
        dmv_x[0] = dmv_y[0] = dmv_x[1] = dmv_y[1] = 0;
        ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
        vc1_b_mc(v, dmv_x, dmv_y, direct, bmvtype);
    } else {
        if (!mb_has_coeffs && !s->mb_intra) {
            /* no coded blocks - effectively skipped */
            ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
            vc1_b_mc(v, dmv_x, dmv_y, direct, bmvtype);
            return 0;
        }
        if (s->mb_intra && !mb_has_coeffs) {
            GET_MQUANT();
            s->current_picture.qscale_table[mb_pos] = mquant;
            s->ac_pred = get_bits1(gb);
            cbp = 0;
            ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
        } else {
            if (bmvtype == BMV_TYPE_INTERPOLATED) {
                GET_MVDATA(dmv_x[0], dmv_y[0]);
                if (!mb_has_coeffs) {
                    /* interpolated skipped block */
                    ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
                    vc1_b_mc(v, dmv_x, dmv_y, direct, bmvtype);
                    return 0;
                }
            }
            ff_vc1_pred_b_mv(v, dmv_x, dmv_y, direct, bmvtype);
            if (!s->mb_intra) {
                vc1_b_mc(v, dmv_x, dmv_y, direct, bmvtype);
            }
            if (s->mb_intra)
                s->ac_pred = get_bits1(gb);
            cbp = get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
            GET_MQUANT();
            s->current_picture.qscale_table[mb_pos] = mquant;
            if (!v->ttmbf && !s->mb_intra && mb_has_coeffs)
                ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
        }
    }
    dst_idx = 0;
    for (i = 0; i < 6; i++) {
        s->dc_val[0][s->block_index[i]] = 0;
        dst_idx += i >> 2;
        val = ((cbp >> (5 - i)) & 1);
        off = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
        v->mb_type[0][s->block_index[i]] = s->mb_intra;
        if (s->mb_intra) {
            /* check if prediction blocks A and C are available */
            v->a_avail = v->c_avail = 0;
            if (i == 2 || i == 3 || !s->first_slice_line)
                v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
            if (i == 1 || i == 3 || s->mb_x)
                v->c_avail = v->mb_type[0][s->block_index[i] - 1];

            vc1_decode_intra_block(v, s->block[i], i, val, mquant,
                                   (i & 4) ? v->codingset2 : v->codingset);
            if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                continue;
            v->vc1dsp.vc1_inv_trans_8x8(s->block[i]);
            if (v->rangeredfrm)
                for (j = 0; j < 64; j++)
                    s->block[i][j] *= 2;
            s->idsp.put_signed_pixels_clamped(s->block[i],
                                              s->dest[dst_idx] + off,
                                              i & 4 ? s->uvlinesize
                                                    : s->linesize);
        } else if (val) {
            int pat = vc1_decode_p_block(v, s->block[i], i, mquant, ttmb,
                                         first_block, s->dest[dst_idx] + off,
                                         (i & 4) ? s->uvlinesize : s->linesize,
                                         CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY), NULL);
            if (pat < 0)
                return pat;
            if (!v->ttmbf && ttmb < 8)
                ttmb = -1;
            first_block = 0;
        }
    }
    return 0;
}

/** Decode one B-frame MB (in interlaced field B picture)
 */
static int vc1_decode_b_mb_intfi(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i, j;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp = 0; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */
    int mb_has_coeffs = 0; /* last_flag */
    int val; /* temp value */
    int first_block = 1;
    int dst_idx, off;
    int fwd;
    int dmv_x[2], dmv_y[2], pred_flag[2];
    int bmvtype = BMV_TYPE_BACKWARD;
    int block_cbp = 0, pat, block_tt = 0;
    int idx_mbmode;

    mquant      = v->pq; /* Lossy initialization */
    s->mb_intra = 0;

    idx_mbmode = get_vlc2(gb, v->mbmode_vlc->table, VC1_IF_MBMODE_VLC_BITS, 2);
    if (idx_mbmode <= 1) { // intra MB
        v->is_intra[s->mb_x] = 0x3f; // Set the bitfield to all 1.
        s->mb_intra          = 1;
        s->current_picture.motion_val[1][s->block_index[0]][0] = 0;
        s->current_picture.motion_val[1][s->block_index[0]][1] = 0;
        s->current_picture.mb_type[mb_pos + v->mb_off]         = MB_TYPE_INTRA;
        GET_MQUANT();
        s->current_picture.qscale_table[mb_pos] = mquant;
        /* Set DC scale - y and c use the same (not sure if necessary here) */
        s->y_dc_scale = s->y_dc_scale_table[FFABS(mquant)];
        s->c_dc_scale = s->c_dc_scale_table[FFABS(mquant)];
        v->s.ac_pred  = v->acpred_plane[mb_pos] = get_bits1(gb);
        mb_has_coeffs = idx_mbmode & 1;
        if (mb_has_coeffs)
            cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_ICBPCY_VLC_BITS, 2);
        dst_idx = 0;
        for (i = 0; i < 6; i++) {
            v->a_avail = v->c_avail          = 0;
            v->mb_type[0][s->block_index[i]] = 1;
            s->dc_val[0][s->block_index[i]]  = 0;
            dst_idx += i >> 2;
            val = ((cbp >> (5 - i)) & 1);
            if (i == 2 || i == 3 || !s->first_slice_line)
                v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
            if (i == 1 || i == 3 || s->mb_x)
                v->c_avail = v->mb_type[0][s->block_index[i] - 1];

            vc1_decode_intra_block(v, s->block[i], i, val, mquant,
                                   (i & 4) ? v->codingset2 : v->codingset);
            if (CONFIG_GRAY && (i > 3) && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                continue;
            v->vc1dsp.vc1_inv_trans_8x8(s->block[i]);
            if (v->rangeredfrm)
                for (j = 0; j < 64; j++)
                    s->block[i][j] <<= 1;
            off  = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
            s->idsp.put_signed_pixels_clamped(s->block[i],
                                              s->dest[dst_idx] + off,
                                              (i & 4) ? s->uvlinesize
                                                      : s->linesize);
        }
    } else {
        s->mb_intra = v->is_intra[s->mb_x] = 0;
        s->current_picture.mb_type[mb_pos + v->mb_off] = MB_TYPE_16x16;
        for (i = 0; i < 6; i++)
            v->mb_type[0][s->block_index[i]] = 0;
        if (v->fmb_is_raw)
            fwd = v->forward_mb_plane[mb_pos] = get_bits1(gb);
        else
            fwd = v->forward_mb_plane[mb_pos];
        if (idx_mbmode <= 5) { // 1-MV
            int interpmvp = 0;
            dmv_x[0]     = dmv_x[1] = dmv_y[0] = dmv_y[1] = 0;
            pred_flag[0] = pred_flag[1] = 0;
            if (fwd)
                bmvtype = BMV_TYPE_FORWARD;
            else {
                bmvtype = decode012(gb);
                switch (bmvtype) {
                case 0:
                    bmvtype = BMV_TYPE_BACKWARD;
                    break;
                case 1:
                    bmvtype = BMV_TYPE_DIRECT;
                    break;
                case 2:
                    bmvtype   = BMV_TYPE_INTERPOLATED;
                    interpmvp = get_bits1(gb);
                }
            }
            v->bmvtype = bmvtype;
            if (bmvtype != BMV_TYPE_DIRECT && idx_mbmode & 1) {
                get_mvdata_interlaced(v, &dmv_x[bmvtype == BMV_TYPE_BACKWARD], &dmv_y[bmvtype == BMV_TYPE_BACKWARD], &pred_flag[bmvtype == BMV_TYPE_BACKWARD]);
            }
            if (interpmvp) {
                get_mvdata_interlaced(v, &dmv_x[1], &dmv_y[1], &pred_flag[1]);
            }
            if (bmvtype == BMV_TYPE_DIRECT) {
                dmv_x[0] = dmv_y[0] = pred_flag[0] = 0;
                dmv_x[1] = dmv_y[1] = pred_flag[0] = 0;
                if (!s->next_picture_ptr->field_picture) {
                    av_log(s->avctx, AV_LOG_ERROR, "Mixed field/frame direct mode not supported\n");
                    return AVERROR_INVALIDDATA;
                }
            }
            ff_vc1_pred_b_mv_intfi(v, 0, dmv_x, dmv_y, 1, pred_flag);
            vc1_b_mc(v, dmv_x, dmv_y, (bmvtype == BMV_TYPE_DIRECT), bmvtype);
            mb_has_coeffs = !(idx_mbmode & 2);
        } else { // 4-MV
            if (fwd)
                bmvtype = BMV_TYPE_FORWARD;
            v->bmvtype  = bmvtype;
            v->fourmvbp = get_vlc2(gb, v->fourmvbp_vlc->table, VC1_4MV_BLOCK_PATTERN_VLC_BITS, 1);
            for (i = 0; i < 4; i++) {
                dmv_x[0] = dmv_y[0] = pred_flag[0] = 0;
                dmv_x[1] = dmv_y[1] = pred_flag[1] = 0;
                if (v->fourmvbp & (8 >> i)) {
                    get_mvdata_interlaced(v, &dmv_x[bmvtype == BMV_TYPE_BACKWARD],
                                             &dmv_y[bmvtype == BMV_TYPE_BACKWARD],
                                         &pred_flag[bmvtype == BMV_TYPE_BACKWARD]);
                }
                ff_vc1_pred_b_mv_intfi(v, i, dmv_x, dmv_y, 0, pred_flag);
                ff_vc1_mc_4mv_luma(v, i, bmvtype == BMV_TYPE_BACKWARD, 0);
            }
            ff_vc1_mc_4mv_chroma(v, bmvtype == BMV_TYPE_BACKWARD);
            mb_has_coeffs = idx_mbmode & 1;
        }
        if (mb_has_coeffs)
            cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
        if (cbp) {
            GET_MQUANT();
        }
        s->current_picture.qscale_table[mb_pos] = mquant;
        if (!v->ttmbf && cbp) {
            ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
        }
        dst_idx = 0;
        for (i = 0; i < 6; i++) {
            s->dc_val[0][s->block_index[i]] = 0;
            dst_idx += i >> 2;
            val = ((cbp >> (5 - i)) & 1);
            off = (i & 4) ? 0 : (i & 1) * 8 + (i & 2) * 4 * s->linesize;
            if (val) {
                pat = vc1_decode_p_block(v, s->block[i], i, mquant, ttmb,
                                         first_block, s->dest[dst_idx] + off,
                                         (i & 4) ? s->uvlinesize : s->linesize,
                                         CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY), &block_tt);
                if (pat < 0)
                    return pat;
                block_cbp |= pat << (i << 2);
                if (!v->ttmbf && ttmb < 8)
                    ttmb = -1;
                first_block = 0;
            }
        }
    }
    v->cbp[s->mb_x]      = block_cbp;
    v->ttblk[s->mb_x]    = block_tt;

    return 0;
}

/** Decode one B-frame MB (in interlaced frame B picture)
 */
static int vc1_decode_b_mb_intfr(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    int i, j;
    int mb_pos = s->mb_x + s->mb_y * s->mb_stride;
    int cbp = 0; /* cbp decoding stuff */
    int mqdiff, mquant; /* MB quantization */
    int ttmb = v->ttfrm; /* MB Transform type */
    int mvsw = 0; /* motion vector switch */
    int mb_has_coeffs = 1; /* last_flag */
    int dmv_x, dmv_y; /* Differential MV components */
    int val; /* temp value */
    int first_block = 1;
    int dst_idx, off;
    int skipped, direct, twomv = 0;
    int block_cbp = 0, pat, block_tt = 0;
    int idx_mbmode = 0, mvbp;
    int stride_y, fieldtx;
    int bmvtype = BMV_TYPE_BACKWARD;
    int dir, dir2;

    mquant = v->pq; /* Lossy initialization */
    s->mb_intra = 0;
    if (v->skip_is_raw)
        skipped = get_bits1(gb);
    else
        skipped = v->s.mbskip_table[mb_pos];

    if (!skipped) {
        idx_mbmode = get_vlc2(gb, v->mbmode_vlc->table, VC1_INTFR_NON4MV_MBMODE_VLC_BITS, 2);
        if (ff_vc1_mbmode_intfrp[0][idx_mbmode][0] == MV_PMODE_INTFR_2MV_FIELD) {
            twomv = 1;
            v->blk_mv_type[s->block_index[0]] = 1;
            v->blk_mv_type[s->block_index[1]] = 1;
            v->blk_mv_type[s->block_index[2]] = 1;
            v->blk_mv_type[s->block_index[3]] = 1;
        } else {
            v->blk_mv_type[s->block_index[0]] = 0;
            v->blk_mv_type[s->block_index[1]] = 0;
            v->blk_mv_type[s->block_index[2]] = 0;
            v->blk_mv_type[s->block_index[3]] = 0;
        }
    }

    if (ff_vc1_mbmode_intfrp[0][idx_mbmode][0] == MV_PMODE_INTFR_INTRA) { // intra MB
        for (i = 0; i < 4; i++) {
            s->mv[0][i][0] = s->current_picture.motion_val[0][s->block_index[i]][0] = 0;
            s->mv[0][i][1] = s->current_picture.motion_val[0][s->block_index[i]][1] = 0;
            s->mv[1][i][0] = s->current_picture.motion_val[1][s->block_index[i]][0] = 0;
            s->mv[1][i][1] = s->current_picture.motion_val[1][s->block_index[i]][1] = 0;
        }
        v->is_intra[s->mb_x] = 0x3f; // Set the bitfield to all 1.
        s->mb_intra          = 1;
        s->current_picture.mb_type[mb_pos] = MB_TYPE_INTRA;
        fieldtx = v->fieldtx_plane[mb_pos] = get_bits1(gb);
        mb_has_coeffs = get_bits1(gb);
        if (mb_has_coeffs)
            cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
        v->s.ac_pred = v->acpred_plane[mb_pos] = get_bits1(gb);
        GET_MQUANT();
        s->current_picture.qscale_table[mb_pos] = mquant;
        /* Set DC scale - y and c use the same (not sure if necessary here) */
        s->y_dc_scale = s->y_dc_scale_table[FFABS(mquant)];
        s->c_dc_scale = s->c_dc_scale_table[FFABS(mquant)];
        dst_idx = 0;
        for (i = 0; i < 6; i++) {
            v->a_avail = v->c_avail          = 0;
            v->mb_type[0][s->block_index[i]] = 1;
            s->dc_val[0][s->block_index[i]]  = 0;
            dst_idx += i >> 2;
            val = ((cbp >> (5 - i)) & 1);
            if (i == 2 || i == 3 || !s->first_slice_line)
                v->a_avail = v->mb_type[0][s->block_index[i] - s->block_wrap[i]];
            if (i == 1 || i == 3 || s->mb_x)
                v->c_avail = v->mb_type[0][s->block_index[i] - 1];

            vc1_decode_intra_block(v, s->block[i], i, val, mquant,
                                   (i & 4) ? v->codingset2 : v->codingset);
            if (CONFIG_GRAY && i > 3 && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                continue;
            v->vc1dsp.vc1_inv_trans_8x8(s->block[i]);
            if (i < 4) {
                stride_y = s->linesize << fieldtx;
                off = (fieldtx) ? ((i & 1) * 8) + ((i & 2) >> 1) * s->linesize : (i & 1) * 8 + 4 * (i & 2) * s->linesize;
            } else {
                stride_y = s->uvlinesize;
                off = 0;
            }
            s->idsp.put_signed_pixels_clamped(s->block[i],
                                              s->dest[dst_idx] + off,
                                              stride_y);
        }
    } else {
        s->mb_intra = v->is_intra[s->mb_x] = 0;

        if (v->dmb_is_raw)
            direct = get_bits1(gb);
        else
            direct = v->direct_mb_plane[mb_pos];

        if (direct) {
            if (s->next_picture_ptr->field_picture)
                av_log(s->avctx, AV_LOG_WARNING, "Mixed frame/field direct mode not supported\n");
            s->mv[0][0][0] = s->current_picture.motion_val[0][s->block_index[0]][0] = scale_mv(s->next_picture.motion_val[1][s->block_index[0]][0], v->bfraction, 0, s->quarter_sample);
            s->mv[0][0][1] = s->current_picture.motion_val[0][s->block_index[0]][1] = scale_mv(s->next_picture.motion_val[1][s->block_index[0]][1], v->bfraction, 0, s->quarter_sample);
            s->mv[1][0][0] = s->current_picture.motion_val[1][s->block_index[0]][0] = scale_mv(s->next_picture.motion_val[1][s->block_index[0]][0], v->bfraction, 1, s->quarter_sample);
            s->mv[1][0][1] = s->current_picture.motion_val[1][s->block_index[0]][1] = scale_mv(s->next_picture.motion_val[1][s->block_index[0]][1], v->bfraction, 1, s->quarter_sample);

            if (twomv) {
                s->mv[0][2][0] = s->current_picture.motion_val[0][s->block_index[2]][0] = scale_mv(s->next_picture.motion_val[1][s->block_index[2]][0], v->bfraction, 0, s->quarter_sample);
                s->mv[0][2][1] = s->current_picture.motion_val[0][s->block_index[2]][1] = scale_mv(s->next_picture.motion_val[1][s->block_index[2]][1], v->bfraction, 0, s->quarter_sample);
                s->mv[1][2][0] = s->current_picture.motion_val[1][s->block_index[2]][0] = scale_mv(s->next_picture.motion_val[1][s->block_index[2]][0], v->bfraction, 1, s->quarter_sample);
                s->mv[1][2][1] = s->current_picture.motion_val[1][s->block_index[2]][1] = scale_mv(s->next_picture.motion_val[1][s->block_index[2]][1], v->bfraction, 1, s->quarter_sample);

                for (i = 1; i < 4; i += 2) {
                    s->mv[0][i][0] = s->current_picture.motion_val[0][s->block_index[i]][0] = s->mv[0][i-1][0];
                    s->mv[0][i][1] = s->current_picture.motion_val[0][s->block_index[i]][1] = s->mv[0][i-1][1];
                    s->mv[1][i][0] = s->current_picture.motion_val[1][s->block_index[i]][0] = s->mv[1][i-1][0];
                    s->mv[1][i][1] = s->current_picture.motion_val[1][s->block_index[i]][1] = s->mv[1][i-1][1];
                }
            } else {
                for (i = 1; i < 4; i++) {
                    s->mv[0][i][0] = s->current_picture.motion_val[0][s->block_index[i]][0] = s->mv[0][0][0];
                    s->mv[0][i][1] = s->current_picture.motion_val[0][s->block_index[i]][1] = s->mv[0][0][1];
                    s->mv[1][i][0] = s->current_picture.motion_val[1][s->block_index[i]][0] = s->mv[1][0][0];
                    s->mv[1][i][1] = s->current_picture.motion_val[1][s->block_index[i]][1] = s->mv[1][0][1];
                }
            }
        }

        if (!direct) {
            if (skipped || !s->mb_intra) {
                bmvtype = decode012(gb);
                switch (bmvtype) {
                case 0:
                    bmvtype = (v->bfraction >= (B_FRACTION_DEN/2)) ? BMV_TYPE_BACKWARD : BMV_TYPE_FORWARD;
                    break;
                case 1:
                    bmvtype = (v->bfraction >= (B_FRACTION_DEN/2)) ? BMV_TYPE_FORWARD : BMV_TYPE_BACKWARD;
                    break;
                case 2:
                    bmvtype  = BMV_TYPE_INTERPOLATED;
                }
            }

            if (twomv && bmvtype != BMV_TYPE_INTERPOLATED)
                mvsw = get_bits1(gb);
        }

        if (!skipped) { // inter MB
            mb_has_coeffs = ff_vc1_mbmode_intfrp[0][idx_mbmode][3];
            if (mb_has_coeffs)
                cbp = 1 + get_vlc2(&v->s.gb, v->cbpcy_vlc->table, VC1_CBPCY_P_VLC_BITS, 2);
            if (!direct) {
                if (bmvtype == BMV_TYPE_INTERPOLATED && twomv) {
                    v->fourmvbp = get_vlc2(gb, v->fourmvbp_vlc->table, VC1_4MV_BLOCK_PATTERN_VLC_BITS, 1);
                } else if (bmvtype == BMV_TYPE_INTERPOLATED || twomv) {
                    v->twomvbp = get_vlc2(gb, v->twomvbp_vlc->table, VC1_2MV_BLOCK_PATTERN_VLC_BITS, 1);
                }
            }

            for (i = 0; i < 6; i++)
                v->mb_type[0][s->block_index[i]] = 0;
            fieldtx = v->fieldtx_plane[mb_pos] = ff_vc1_mbmode_intfrp[0][idx_mbmode][1];
            /* for all motion vector read MVDATA and motion compensate each block */
            dst_idx = 0;
            if (direct) {
                if (twomv) {
                    for (i = 0; i < 4; i++) {
                        ff_vc1_mc_4mv_luma(v, i, 0, 0);
                        ff_vc1_mc_4mv_luma(v, i, 1, 1);
                    }
                    ff_vc1_mc_4mv_chroma4(v, 0, 0, 0);
                    ff_vc1_mc_4mv_chroma4(v, 1, 1, 1);
                } else {
                    ff_vc1_mc_1mv(v, 0);
                    ff_vc1_interp_mc(v);
                }
            } else if (twomv && bmvtype == BMV_TYPE_INTERPOLATED) {
                mvbp = v->fourmvbp;
                for (i = 0; i < 4; i++) {
                    dir = i==1 || i==3;
                    dmv_x = dmv_y = 0;
                    val = ((mvbp >> (3 - i)) & 1);
                    if (val)
                        get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                    j = i > 1 ? 2 : 0;
                    ff_vc1_pred_mv_intfr(v, j, dmv_x, dmv_y, 2, v->range_x, v->range_y, v->mb_type[0], dir);
                    ff_vc1_mc_4mv_luma(v, j, dir, dir);
                    ff_vc1_mc_4mv_luma(v, j+1, dir, dir);
                }

                ff_vc1_mc_4mv_chroma4(v, 0, 0, 0);
                ff_vc1_mc_4mv_chroma4(v, 1, 1, 1);
            } else if (bmvtype == BMV_TYPE_INTERPOLATED) {
                mvbp = v->twomvbp;
                dmv_x = dmv_y = 0;
                if (mvbp & 2)
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);

                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], 0);
                ff_vc1_mc_1mv(v, 0);

                dmv_x = dmv_y = 0;
                if (mvbp & 1)
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);

                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], 1);
                ff_vc1_interp_mc(v);
            } else if (twomv) {
                dir = bmvtype == BMV_TYPE_BACKWARD;
                dir2 = dir;
                if (mvsw)
                    dir2 = !dir;
                mvbp = v->twomvbp;
                dmv_x = dmv_y = 0;
                if (mvbp & 2)
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 2, v->range_x, v->range_y, v->mb_type[0], dir);

                dmv_x = dmv_y = 0;
                if (mvbp & 1)
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);
                ff_vc1_pred_mv_intfr(v, 2, dmv_x, dmv_y, 2, v->range_x, v->range_y, v->mb_type[0], dir2);

                if (mvsw) {
                    for (i = 0; i < 2; i++) {
                        s->mv[dir][i+2][0] = s->mv[dir][i][0] = s->current_picture.motion_val[dir][s->block_index[i+2]][0] = s->current_picture.motion_val[dir][s->block_index[i]][0];
                        s->mv[dir][i+2][1] = s->mv[dir][i][1] = s->current_picture.motion_val[dir][s->block_index[i+2]][1] = s->current_picture.motion_val[dir][s->block_index[i]][1];
                        s->mv[dir2][i+2][0] = s->mv[dir2][i][0] = s->current_picture.motion_val[dir2][s->block_index[i]][0] = s->current_picture.motion_val[dir2][s->block_index[i+2]][0];
                        s->mv[dir2][i+2][1] = s->mv[dir2][i][1] = s->current_picture.motion_val[dir2][s->block_index[i]][1] = s->current_picture.motion_val[dir2][s->block_index[i+2]][1];
                    }
                } else {
                    ff_vc1_pred_mv_intfr(v, 0, 0, 0, 2, v->range_x, v->range_y, v->mb_type[0], !dir);
                    ff_vc1_pred_mv_intfr(v, 2, 0, 0, 2, v->range_x, v->range_y, v->mb_type[0], !dir);
                }

                ff_vc1_mc_4mv_luma(v, 0, dir, 0);
                ff_vc1_mc_4mv_luma(v, 1, dir, 0);
                ff_vc1_mc_4mv_luma(v, 2, dir2, 0);
                ff_vc1_mc_4mv_luma(v, 3, dir2, 0);
                ff_vc1_mc_4mv_chroma4(v, dir, dir2, 0);
            } else {
                dir = bmvtype == BMV_TYPE_BACKWARD;

                mvbp = ff_vc1_mbmode_intfrp[0][idx_mbmode][2];
                dmv_x = dmv_y = 0;
                if (mvbp)
                    get_mvdata_interlaced(v, &dmv_x, &dmv_y, 0);

                ff_vc1_pred_mv_intfr(v, 0, dmv_x, dmv_y, 1, v->range_x, v->range_y, v->mb_type[0], dir);
                v->blk_mv_type[s->block_index[0]] = 1;
                v->blk_mv_type[s->block_index[1]] = 1;
                v->blk_mv_type[s->block_index[2]] = 1;
                v->blk_mv_type[s->block_index[3]] = 1;
                ff_vc1_pred_mv_intfr(v, 0, 0, 0, 2, v->range_x, v->range_y, 0, !dir);
                for (i = 0; i < 2; i++) {
                    s->mv[!dir][i+2][0] = s->mv[!dir][i][0] = s->current_picture.motion_val[!dir][s->block_index[i+2]][0] = s->current_picture.motion_val[!dir][s->block_index[i]][0];
                    s->mv[!dir][i+2][1] = s->mv[!dir][i][1] = s->current_picture.motion_val[!dir][s->block_index[i+2]][1] = s->current_picture.motion_val[!dir][s->block_index[i]][1];
                }
                ff_vc1_mc_1mv(v, dir);
            }

            if (cbp)
                GET_MQUANT();  // p. 227
            s->current_picture.qscale_table[mb_pos] = mquant;
            if (!v->ttmbf && cbp)
                ttmb = get_vlc2(gb, ff_vc1_ttmb_vlc[v->tt_index].table, VC1_TTMB_VLC_BITS, 2);
            for (i = 0; i < 6; i++) {
                s->dc_val[0][s->block_index[i]] = 0;
                dst_idx += i >> 2;
                val = ((cbp >> (5 - i)) & 1);
                if (!fieldtx)
                    off = (i & 4) ? 0 : ((i & 1) * 8 + (i & 2) * 4 * s->linesize);
                else
                    off = (i & 4) ? 0 : ((i & 1) * 8 + ((i > 1) * s->linesize));
                if (val) {
                    pat = vc1_decode_p_block(v, s->block[i], i, mquant, ttmb,
                                             first_block, s->dest[dst_idx] + off,
                                             (i & 4) ? s->uvlinesize : (s->linesize << fieldtx),
                                             CONFIG_GRAY && (i & 4) && (s->avctx->flags & AV_CODEC_FLAG_GRAY), &block_tt);
                    if (pat < 0)
                        return pat;
                    block_cbp |= pat << (i << 2);
                    if (!v->ttmbf && ttmb < 8)
                        ttmb = -1;
                    first_block = 0;
                }
            }

        } else { // skipped
            dir = 0;
            for (i = 0; i < 6; i++) {
                v->mb_type[0][s->block_index[i]] = 0;
                s->dc_val[0][s->block_index[i]] = 0;
            }
            s->current_picture.mb_type[mb_pos]      = MB_TYPE_SKIP;
            s->current_picture.qscale_table[mb_pos] = 0;
            v->blk_mv_type[s->block_index[0]] = 0;
            v->blk_mv_type[s->block_index[1]] = 0;
            v->blk_mv_type[s->block_index[2]] = 0;
            v->blk_mv_type[s->block_index[3]] = 0;

            if (!direct) {
                if (bmvtype == BMV_TYPE_INTERPOLATED) {
                    ff_vc1_pred_mv_intfr(v, 0, 0, 0, 1, v->range_x, v->range_y, v->mb_type[0], 0);
                    ff_vc1_pred_mv_intfr(v, 0, 0, 0, 1, v->range_x, v->range_y, v->mb_type[0], 1);
                } else {
                    dir = bmvtype == BMV_TYPE_BACKWARD;
                    ff_vc1_pred_mv_intfr(v, 0, 0, 0, 1, v->range_x, v->range_y, v->mb_type[0], dir);
                    if (mvsw) {
                        int dir2 = dir;
                        if (mvsw)
                            dir2 = !dir;
                        for (i = 0; i < 2; i++) {
                            s->mv[dir][i+2][0] = s->mv[dir][i][0] = s->current_picture.motion_val[dir][s->block_index[i+2]][0] = s->current_picture.motion_val[dir][s->block_index[i]][0];
                            s->mv[dir][i+2][1] = s->mv[dir][i][1] = s->current_picture.motion_val[dir][s->block_index[i+2]][1] = s->current_picture.motion_val[dir][s->block_index[i]][1];
                            s->mv[dir2][i+2][0] = s->mv[dir2][i][0] = s->current_picture.motion_val[dir2][s->block_index[i]][0] = s->current_picture.motion_val[dir2][s->block_index[i+2]][0];
                            s->mv[dir2][i+2][1] = s->mv[dir2][i][1] = s->current_picture.motion_val[dir2][s->block_index[i]][1] = s->current_picture.motion_val[dir2][s->block_index[i+2]][1];
                        }
                    } else {
                        v->blk_mv_type[s->block_index[0]] = 1;
                        v->blk_mv_type[s->block_index[1]] = 1;
                        v->blk_mv_type[s->block_index[2]] = 1;
                        v->blk_mv_type[s->block_index[3]] = 1;
                        ff_vc1_pred_mv_intfr(v, 0, 0, 0, 2, v->range_x, v->range_y, 0, !dir);
                        for (i = 0; i < 2; i++) {
                            s->mv[!dir][i+2][0] = s->mv[!dir][i][0] = s->current_picture.motion_val[!dir][s->block_index[i+2]][0] = s->current_picture.motion_val[!dir][s->block_index[i]][0];
                            s->mv[!dir][i+2][1] = s->mv[!dir][i][1] = s->current_picture.motion_val[!dir][s->block_index[i+2]][1] = s->current_picture.motion_val[!dir][s->block_index[i]][1];
                        }
                    }
                }
            }

            ff_vc1_mc_1mv(v, dir);
            if (direct || bmvtype == BMV_TYPE_INTERPOLATED) {
                ff_vc1_interp_mc(v);
            }
            v->fieldtx_plane[mb_pos] = 0;
        }
    }
    v->cbp[s->mb_x]      = block_cbp;
    v->ttblk[s->mb_x]    = block_tt;

    return 0;
}

static inline int vc1_decode_i_mb(VC1IMBCtx *mbctx,
                                  VC1IntraBlkCtx *blkctx,
                                  int blkidx[BLOCKIDX_MAX],
                                  GetBitContext *gb)
{
    VC1StoredBlkCtx *curr_sblkctx = mbctx->s_blkctx + blkidx[BLOCKIDX_Y0];
    unsigned int cbpcy;
    int ret;

    mbctx->s_mbctx[blkidx[MBIDX]].loopfilter_blk = 0;
    mbctx->s_mbctx[blkidx[MBIDX]].loopfilter_subblk = 0;

    switch (mbctx->mbtype) {
    case MB_I:
        cbpcy = get_vlc2(gb, mbctx->cbpcy_vlc->table, 8, 2); // CBPCY
        blkctx->use_ac_pred = get_bits1(gb); // ACPRED
        blkctx->use_cbpcy_pred = 1;
        break;

    case MB_P:
        cbpcy = mbctx->cbpcy;
        break;

    default:
        av_assert0(0);
    }

    blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_LUMA];
    blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_LUMA];
    if (CONFIG_GRAY)
        blkctx->skip_output = 0;

    /* decode block Y0 */
    curr_sblkctx[0].is_coded = cbpcy >> 5 & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0],
                                     blkidx[BLOCKIDX_T2],
                                     blkidx[BLOCKIDX_LT3],
                                     blkidx[BLOCKIDX_L1],
                                     gb);
    if (ret < 0)
        return ret;

    /* decode block Y1 */
    curr_sblkctx[2].is_coded = cbpcy >> 4 & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0] + 2,
                                     blkidx[BLOCKIDX_T3],
                                     blkidx[BLOCKIDX_T2],
                                     blkidx[BLOCKIDX_Y0],
                                     gb);
    if (ret < 0)
        return ret;

    /* decode block Y2 */
    curr_sblkctx[1].is_coded = cbpcy >> 3 & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0] + 1,
                                     blkidx[BLOCKIDX_Y0],
                                     blkidx[BLOCKIDX_L1],
                                     blkidx[BLOCKIDX_L3],
                                     gb);
    if (ret < 0)
        return ret;

    /* decode block Y3 */
    curr_sblkctx[3].is_coded = cbpcy >> 2 & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0] + 3,
                                     blkidx[BLOCKIDX_Y0] + 2,
                                     blkidx[BLOCKIDX_Y0],
                                     blkidx[BLOCKIDX_Y0] + 1,
                                     gb);
    if (ret < 0)
        return ret;

    blkctx->ac_coding_set = &mbctx->ac_coding_set[COMPONENT_TYPE_CHROMA];
    blkctx->dc_diff_vlc = &mbctx->dc_diff_vlc[COMPONENT_TYPE_CHROMA];
    blkctx->use_cbpcy_pred = 0;
    if (CONFIG_GRAY && mbctx->codec_flag_gray)
        blkctx->skip_output = 1;

    /* decode block Cb */
    curr_sblkctx[4].is_coded = cbpcy >> 1 & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0] + 4,
                                     blkidx[BLOCKIDX_CB_T],
                                     blkidx[BLOCKIDX_CB_LT],
                                     blkidx[BLOCKIDX_CB_L],
                                     gb);
    if (ret < 0)
        return ret;

    /* decode block Cr */
    curr_sblkctx[5].is_coded = cbpcy & 1;
    ret = vc1_decode_intra_block_new((VC1MBCtx*)mbctx,
                                     blkctx,
                                     blkidx[MBIDX],
                                     blkidx[BLOCKIDX_Y0] + 5,
                                     blkidx[BLOCKIDX_CR_T],
                                     blkidx[BLOCKIDX_CR_LT],
                                     blkidx[BLOCKIDX_CR_L],
                                     gb);
    if (ret < 0)
        return ret;

    return 0;
}

/** Decode blocks of I-frame
 */
static void vc1_decode_i_blocks(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    VC1IPictCtx *pict = (VC1IPictCtx*)v->pict;
    VC1IMBCtx *mbctx = (VC1IMBCtx*)&v->mbctx;
    VC1IntraBlkCtx blkctx;
    int blkidx[BLOCKIDX_MAX];
    int topleft_mbidx, toptopleft_mbidx;
    int curr_blkidx, topleft_blkidx, left_blkidx;
    uint8_t **dest;
    GetBitContext *gb = &v->s.gb;
    int k, j, i;
    int mb_pos;

    mbctx->mbtype = MB_I;
    mbctx->vc1dsp = &v->vc1dsp;
    mbctx->vc1_v_loop_filter[0] = ff_vc1_loop_filter_noop;
    mbctx->vc1_v_loop_filter[1] = mbctx->vc1dsp->vc1_v_loop_filter4;
    mbctx->vc1_v_loop_filter[2] = mbctx->vc1dsp->vc1_v_loop_filter4;
    mbctx->vc1_v_loop_filter[3] = mbctx->vc1dsp->vc1_v_loop_filter8;
    mbctx->vc1_h_loop_filter[0] = ff_vc1_loop_filter_noop;
    mbctx->vc1_h_loop_filter[1] = mbctx->vc1dsp->vc1_h_loop_filter4;
    mbctx->vc1_h_loop_filter[2] = mbctx->vc1dsp->vc1_h_loop_filter4;
    mbctx->vc1_h_loop_filter[3] = mbctx->vc1dsp->vc1_h_loop_filter8;
    mbctx->linesize[COMPONENT_TYPE_LUMA] = s->current_picture.f->linesize[0];
    mbctx->linesize[COMPONENT_TYPE_CHROMA] = s->current_picture.f->linesize[1];
    mbctx->s_mbctx[0].dest[COMPONENT_LUMA] = s->current_picture.f->data[0];
    mbctx->s_mbctx[0].dest[COMPONENT_CB] = s->current_picture.f->data[1];
    mbctx->s_mbctx[0].dest[COMPONENT_CR] = s->current_picture.f->data[2];
    mbctx->put_pixels = ((VC1SimpleSeqCtx*)v->seq)->overlap && pict->pquant >= 9 ?
                        s->idsp.put_signed_pixels_clamped :
                        s->idsp.put_pixels_clamped;
    mbctx->use_overlap_xfrm = ((VC1SimpleSeqCtx*)v->seq)->overlap && pict->pquant >= 9;
    mbctx->use_loopfilter = v->seq->profile == PROFILE_MAIN ? ((VC1MainSeqCtx*)v->seq)->loopfilter : 0;
    if (CONFIG_GRAY)
        mbctx->codec_flag_gray = !!(v->avctx->flags & AV_CODEC_FLAG_GRAY); // TODO: move out of decoding loop
    mbctx->s_blkctx[-1].btype = BLOCK_INTRA_OOB;
    mbctx->s_blkctx[-1].dc_pred = v->overlap && v->pq >= 9 ? 0 : 1024;
    mbctx->dqedge = DQEDGE_NONE;

    blkctx.btype = BLOCK_INTRA;
    blkctx.vc1dsp = &v->vc1dsp;
    blkctx.idsp = &s->idsp;
    blkctx.zz_8x8 = &pict->zz_8x8;
    blkctx.mquant = ((VC1SimplePictCtx*)v->pict)->pquant;
    blkctx.double_quant = 2 * blkctx.mquant + pict->halfqp;
    blkctx.quant_scale = blkctx.mquant * !pict->pquantizer;
    blkctx.fasttx = ((VC1SimpleSeqCtx*)v->seq)->res_fasttx;
    blkctx.ac_level_code_size = &pict->ac_level_code_size;
    blkctx.ac_run_code_size = &pict->ac_run_code_size;
    blkctx.esc_mode3_vlc = pict->pquant < 8;
    blkctx.s_blkctx = mbctx->s_blkctx;
    blkctx.block = mbctx->block;
    if (!CONFIG_GRAY)
        blkctx.skip_output = 0;

    //do frame decode
    i = 0;
    s->mb_x = s->mb_y = 0;
    s->mb_intra         = 1;
    s->first_slice_line = 1;

    for (s->mb_y = s->start_mb_y; s->mb_y < s->end_mb_y; s->mb_y++) {
        s->mb_x = 0;
        init_block_index(v);

        for (; s->mb_x < v->end_mb_x; s->mb_x++) {
            update_block_index((VC1MBCtx*)mbctx, blkidx, v->end_mb_x, v->end_mb_x * (s->end_mb_y - s->start_mb_y), i);
            ff_update_block_index(s);

            s->bdsp.clear_blocks(v->block[v->cur_blk_idx][0]);
            mb_pos = s->mb_x + s->mb_y * s->mb_width;
            s->current_picture.mb_type[mb_pos]                     = MB_TYPE_INTRA;
//            s->current_picture.qscale_table[mb_pos]                = v->pq;
            for (int i = 0; i < 4; i++) {
                s->current_picture.motion_val[1][s->block_index[i]][0] = 0;
                s->current_picture.motion_val[1][s->block_index[i]][1] = 0;
            }

            // do actual MB decoding and displaying
            v->mb_type[0][s->block_index[0]] = 1;
            v->mb_type[0][s->block_index[1]] = 1;
            v->mb_type[0][s->block_index[2]] = 1;
            v->mb_type[0][s->block_index[3]] = 1;
            v->mb_type[0][s->block_index[4]] = 1;
            v->mb_type[0][s->block_index[5]] = 1;

            vc1_decode_i_mb(mbctx,
                            &blkctx,
                            blkidx,
                            gb);

            topleft_mbidx = blkidx[MBIDX] < v->end_mb_x + 1 ? blkidx[MBIDX] + v->end_mb_x + 2 : blkidx[MBIDX] - v->end_mb_x - 1;
            toptopleft_mbidx = blkidx[MBIDX] < 2 * v->end_mb_x + 1 ? blkidx[MBIDX] + 2 : blkidx[MBIDX] - 2 * v->end_mb_x - 1;
            curr_blkidx = blkidx[BLOCKIDX_Y0];
            topleft_blkidx = 6 * ((i + 1) % (v->end_mb_x + 2));
            left_blkidx = 6 * ((i - 1) % (v->end_mb_x + 2));

            ff_vc1_h_overlap_filter((VC1MBCtx*)mbctx, curr_blkidx, left_blkidx);

            if (i > 0)
                ff_vc1_v_overlap_filter((VC1MBCtx*)mbctx, left_blkidx, topleft_blkidx);

            if (i > v->end_mb_x) {
                dest = mbctx->s_mbctx[topleft_mbidx].dest;

                vc1_put_pixels((VC1MBCtx*)mbctx, topleft_blkidx, dest);

                ff_vc1_v_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[topleft_mbidx].loopfilter_blk, dest);
            }

            if (i > 2 * v->end_mb_x) {
                dest = mbctx->s_mbctx[toptopleft_mbidx].dest;

                ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleft_mbidx].loopfilter_blk >> 2, dest);
            }

            if (s->mb_x == v->end_mb_x - 1 && s->mb_y == s->end_mb_y - 1) {
                int tail = 2 * v->end_mb_x;

                do {
                    i++;
                    update_block_index((VC1MBCtx*)mbctx, blkidx, v->end_mb_x, v->end_mb_x * (s->end_mb_y - s->start_mb_y), i);

                    topleft_mbidx = blkidx[MBIDX] < v->end_mb_x + 1 ? blkidx[MBIDX] + v->end_mb_x + 2 : blkidx[MBIDX] - v->end_mb_x - 1;
                    toptopleft_mbidx = blkidx[MBIDX] < 2 * v->end_mb_x + 1 ? blkidx[MBIDX] + 2 : blkidx[MBIDX] - 2 * v->end_mb_x - 1;
                    curr_blkidx = blkidx[BLOCKIDX_Y0];
                    topleft_blkidx = 6 * ((i + 1) % (v->end_mb_x + 2));
                    left_blkidx = 6 * ((i - 1) % (v->end_mb_x + 2));

                    if (tail == 2 * v->end_mb_x)
                        ff_vc1_v_overlap_filter((VC1MBCtx*)mbctx, left_blkidx, topleft_blkidx);

                    if (tail > v->end_mb_x - 1) {
                        dest = mbctx->s_mbctx[topleft_mbidx].dest;

                        vc1_put_pixels((VC1MBCtx*)mbctx, topleft_blkidx, dest);

                        ff_vc1_v_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[topleft_mbidx].loopfilter_blk, dest);
                    }

                    dest = mbctx->s_mbctx[toptopleft_mbidx].dest;

                    ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleft_mbidx].loopfilter_blk >> 2, dest);
                } while (tail--);
            }

            if (mbctx->use_overlap_xfrm) {
                if (v->rangeredfrm)
                    /* This is not correct. Range reduction should occur after the frame is loop filtered */
                    for (k = 0; k < 6; k++)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[k]][j] *= 2;
            } else {
                if (v->rangeredfrm)
                    for (k = 0; k < 6; k++)
                        for (j = 0; j < 64; j++)
                            v->block[v->cur_blk_idx][block_map[k]][j] = (v->block[v->cur_blk_idx][block_map[k]][j] - 64) * 2;
            }

            if (get_bits_left(gb) < 0) {
                ff_er_add_slice(&s->er, 0, 0, s->mb_x, s->mb_y, ER_MB_ERROR);
                av_log(s->avctx, AV_LOG_ERROR, "Bits overconsumption: %i > %i\n",
                       get_bits_count(gb), gb->size_in_bits);
                return;
            }

            v->topleft_blk_idx = (v->topleft_blk_idx + 1) % (v->end_mb_x + 2);
            v->top_blk_idx = (v->top_blk_idx + 1) % (v->end_mb_x + 2);
            v->left_blk_idx = (v->left_blk_idx + 1) % (v->end_mb_x + 2);
            v->cur_blk_idx = (v->cur_blk_idx + 1) % (v->end_mb_x + 2);

            i++;
        }

        if (!v->s.loop_filter)
            ff_mpeg_draw_horiz_band(s, s->mb_y * 16, 16);
        else if (s->mb_y)
            ff_mpeg_draw_horiz_band(s, (s->mb_y - 1) * 16, 16);

        s->first_slice_line = 0;
    }

    if (v->s.loop_filter)
        ff_mpeg_draw_horiz_band(s, (s->end_mb_y - 1) * 16, 16);

    /* This is intentionally mb_height and not end_mb_y - unlike in advanced
     * profile, these only differ are when decoding MSS2 rectangles. */
    ff_er_add_slice(&s->er, 0, 0, s->mb_width - 1, s->mb_height - 1, ER_MB_END);
}

/** Decode blocks of I-frame for advanced profile
 */
static int vc1_decode_i_blocks_adv(VC1Context *v)
{
    int k;
    MpegEncContext *s = &v->s;
    int cbp, val;
    uint8_t *coded_val;
    int mb_pos;
    int mquant;
    int mqdiff;
    GetBitContext *gb = &s->gb;

    if (get_bits_left(gb) <= 1)
        return AVERROR_INVALIDDATA;

    /* select coding mode used for VLC tables selection */
    switch (v->y_ac_table_index) {
    case 0:
        v->codingset = (v->pqindex <= 8) ? CS_HIGH_RATE_INTRA : CS_LOW_MOT_INTRA;
        break;
    case 1:
        v->codingset = CS_HIGH_MOT_INTRA;
        break;
    case 2:
        v->codingset = CS_MID_RATE_INTRA;
        break;
    }

    switch (v->c_ac_table_index) {
    case 0:
        v->codingset2 = (v->pqindex <= 8) ? CS_HIGH_RATE_INTER : CS_LOW_MOT_INTER;
        break;
    case 1:
        v->codingset2 = CS_HIGH_MOT_INTER;
        break;
    case 2:
        v->codingset2 = CS_MID_RATE_INTER;
        break;
    }

    // do frame decode
    s->mb_x             = s->mb_y = 0;
    s->mb_intra         = 1;
    s->first_slice_line = 1;
    s->mb_y             = s->start_mb_y;
    if (s->start_mb_y) {
        s->mb_x = 0;
        init_block_index(v);
        memset(&s->coded_block[s->block_index[0] - s->b8_stride], 0,
               (1 + s->b8_stride) * sizeof(*s->coded_block));
    }
    for (; s->mb_y < s->end_mb_y; s->mb_y++) {
        s->mb_x = 0;
        init_block_index(v);
        for (;s->mb_x < s->mb_width; s->mb_x++) {
            mquant = v->pq;
            ff_update_block_index(s);
            s->bdsp.clear_blocks(v->block[v->cur_blk_idx][0]);
            mb_pos = s->mb_x + s->mb_y * s->mb_stride;
            s->current_picture.mb_type[mb_pos + v->mb_off]                         = MB_TYPE_INTRA;
            for (int i = 0; i < 4; i++) {
                s->current_picture.motion_val[1][s->block_index[i] + v->blocks_off][0] = 0;
                s->current_picture.motion_val[1][s->block_index[i] + v->blocks_off][1] = 0;
            }

            // do actual MB decoding and displaying
            if (v->fieldtx_is_raw)
                v->fieldtx_plane[mb_pos] = get_bits1(&v->s.gb);
            cbp = get_vlc2(&v->s.gb, ff_msmp4_mb_i_vlc.table, MB_INTRA_VLC_BITS, 2);
            if (v->acpred_is_raw)
                v->s.ac_pred = get_bits1(&v->s.gb);
            else
                v->s.ac_pred = v->acpred_plane[mb_pos];

            if (v->condover == CONDOVER_SELECT && v->overflg_is_raw)
                v->over_flags_plane[mb_pos] = get_bits1(&v->s.gb);

            GET_MQUANT();

            s->current_picture.qscale_table[mb_pos] = mquant;
            /* Set DC scale - y and c use the same */
            s->y_dc_scale = s->y_dc_scale_table[FFABS(mquant)];
            s->c_dc_scale = s->c_dc_scale_table[FFABS(mquant)];

            for (k = 0; k < 6; k++) {
                v->mb_type[0][s->block_index[k]] = 1;

                val = ((cbp >> (5 - k)) & 1);

                if (k < 4) {
                    int pred   = vc1_coded_block_pred(&v->s, k, &coded_val);
                    val        = val ^ pred;
                    *coded_val = val;
                }
                cbp |= val << (5 - k);

                v->a_avail = !s->first_slice_line || (k == 2 || k == 3);
                v->c_avail = !!s->mb_x || (k == 1 || k == 3);

                vc1_decode_i_block_adv(v, v->block[v->cur_blk_idx][block_map[k]], k, val,
                                       (k < 4) ? v->codingset : v->codingset2, mquant);

                if (CONFIG_GRAY && k > 3 && (s->avctx->flags & AV_CODEC_FLAG_GRAY))
                    continue;
                v->vc1dsp.vc1_inv_trans_8x8(v->block[v->cur_blk_idx][block_map[k]]);
            }

            if (v->overlap && (v->pq >= 9 || v->condover != CONDOVER_NONE))
                ff_vc1_i_overlap_filter(v);
            vc1_put_blocks_clamped(v, 1);
            if (v->s.loop_filter)
                ff_vc1_i_loop_filter(v);

            if (get_bits_left(&s->gb) < 0) {
                // TODO: may need modification to handle slice coding
                ff_er_add_slice(&s->er, 0, s->start_mb_y, s->mb_x, s->mb_y, ER_MB_ERROR);
                av_log(s->avctx, AV_LOG_ERROR, "Bits overconsumption: %i > %i\n",
                       get_bits_count(&s->gb), s->gb.size_in_bits);
                return 0;
            }
            inc_blk_idx(v->topleft_blk_idx);
            inc_blk_idx(v->top_blk_idx);
            inc_blk_idx(v->left_blk_idx);
            inc_blk_idx(v->cur_blk_idx);
        }
        if (!v->s.loop_filter)
            ff_mpeg_draw_horiz_band(s, s->mb_y * 16, 16);
        else if (s->mb_y)
            ff_mpeg_draw_horiz_band(s, (s->mb_y-1) * 16, 16);
        s->first_slice_line = 0;
    }

    if (v->s.loop_filter)
        ff_mpeg_draw_horiz_band(s, (s->end_mb_y - 1) * 16, 16);
    ff_er_add_slice(&s->er, 0, s->start_mb_y << v->field_mode, s->mb_width - 1,
                    (s->end_mb_y << v->field_mode) - 1, ER_MB_END);
    return 0;
}

static void vc1_decode_p_blocks(VC1Context *v)
{
    MpegEncContext *s = &v->s;
    GetBitContext *gb = &s->gb;
    VC1PPictCtx *pict = (VC1PPictCtx*)v->pict;
    VC1PMBCtx *mbctx = (VC1PMBCtx*)&v->mbctx;
    uint8_t **dest;
    VC1IntraBlkCtx intra_blkctx;
    VC1InterBlkCtx inter_blkctx;
    int blkidx[BLOCKIDX_MAX];
    int topleft_mbidx, toptopleft_mbidx, toptopleftleft_mbidx;
    int curr_blkidx, topleft_blkidx, left_blkidx;
    int apply_loop_filter;
    int i;

    /* select coding mode used for VLC tables selection */
    switch (v->c_ac_table_index) {
    case 0:
        v->codingset = (v->pqindex <= 8) ? CS_HIGH_RATE_INTRA : CS_LOW_MOT_INTRA;
        break;
    case 1:
        v->codingset = CS_HIGH_MOT_INTRA;
        break;
    case 2:
        v->codingset = CS_MID_RATE_INTRA;
        break;
    }

    switch (v->c_ac_table_index) {
    case 0:
        v->codingset2 = (v->pqindex <= 8) ? CS_HIGH_RATE_INTER : CS_LOW_MOT_INTER;
        break;
    case 1:
        v->codingset2 = CS_HIGH_MOT_INTER;
        break;
    case 2:
        v->codingset2 = CS_MID_RATE_INTER;
        break;
    }

    for (int i = 0; i < 16; i++) {
        mbctx->put_pixels_bicubic_16x16[i] = v->vc1dsp.put_vc1_mspel_pixels_tab[0][i];
        mbctx->put_pixels_bicubic_8x8[i] = v->vc1dsp.put_vc1_mspel_pixels_tab[1][i];
    }

    mbctx->put_pixels_bilin[0] = s->hdsp.put_pixels_tab[0][0];
    mbctx->put_pixels_bilin[1] = s->hdsp.put_pixels_tab[1][0];
    mbctx->put_pixels_bilin[2] = s->hdsp.put_pixels_tab[0][1];
    mbctx->put_pixels_bilin[3] = s->hdsp.put_pixels_tab[1][1];
    mbctx->put_pixels_bilin[4] = s->hdsp.put_no_rnd_pixels_tab[0][0];
    mbctx->put_pixels_bilin[5] = s->hdsp.put_no_rnd_pixels_tab[1][0];
    mbctx->put_pixels_bilin[6] = s->hdsp.put_no_rnd_pixels_tab[0][1];
    mbctx->put_pixels_bilin[7] = s->hdsp.put_no_rnd_pixels_tab[1][1];
    mbctx->put_pixels_bilin[8] = s->hdsp.put_pixels_tab[0][2];
    mbctx->put_pixels_bilin[9] = s->hdsp.put_pixels_tab[1][2];
    mbctx->put_pixels_bilin[10] = s->hdsp.put_pixels_tab[0][3];
    mbctx->put_pixels_bilin[11] = s->hdsp.put_pixels_tab[1][3];
    mbctx->put_pixels_bilin[12] = s->hdsp.put_no_rnd_pixels_tab[0][2];
    mbctx->put_pixels_bilin[13] = s->hdsp.put_no_rnd_pixels_tab[1][2];
    mbctx->put_pixels_bilin[14] = s->hdsp.put_no_rnd_pixels_tab[0][3];
    mbctx->put_pixels_bilin[15] = s->hdsp.put_no_rnd_pixels_tab[1][3];

    mbctx->mbtype = MB_P;
    mbctx->vc1dsp = &v->vc1dsp;
    mbctx->hdsp = &s->hdsp;
    mbctx->emulated_edge_mc = s->vdsp.emulated_edge_mc;
    mbctx->vc1_v_loop_filter[0] = ff_vc1_loop_filter_noop;
    mbctx->vc1_v_loop_filter[1] = mbctx->vc1dsp->vc1_v_loop_filter4;
    mbctx->vc1_v_loop_filter[2] = mbctx->vc1dsp->vc1_v_loop_filter4;
    mbctx->vc1_v_loop_filter[3] = mbctx->vc1dsp->vc1_v_loop_filter8;
    mbctx->vc1_h_loop_filter[0] = ff_vc1_loop_filter_noop;
    mbctx->vc1_h_loop_filter[1] = mbctx->vc1dsp->vc1_h_loop_filter4;
    mbctx->vc1_h_loop_filter[2] = mbctx->vc1dsp->vc1_h_loop_filter4;
    mbctx->vc1_h_loop_filter[3] = mbctx->vc1dsp->vc1_h_loop_filter8;
    mbctx->put_pixels = s->idsp.put_signed_pixels_clamped;
    mbctx->linesize[COMPONENT_TYPE_LUMA] = s->current_picture.f->linesize[0];
    mbctx->linesize[COMPONENT_TYPE_CHROMA] = s->current_picture.f->linesize[1];
    mbctx->s_mbctx[0].dest[COMPONENT_LUMA] = s->current_picture.f->data[0];
    mbctx->s_mbctx[0].dest[COMPONENT_CB] = s->current_picture.f->data[1];
    mbctx->s_mbctx[0].dest[COMPONENT_CR] = s->current_picture.f->data[2];
    mbctx->mb_aligned_width_qpel = s->mb_width * 64;
    mbctx->mb_aligned_height_qpel = s->mb_height * 64;
    mbctx->use_overlap_xfrm = ((VC1SimpleSeqCtx*)v->seq)->overlap && pict->pquant >= 9;
    mbctx->use_loopfilter = v->seq->profile == PROFILE_MAIN ? ((VC1MainSeqCtx*)v->seq)->loopfilter : 0;
    if (CONFIG_GRAY)
        mbctx->codec_flag_gray = !!(v->avctx->flags & AV_CODEC_FLAG_GRAY); // TODO: move out of decoding loop
    mbctx->s_blkctx[-1].btype = BLOCK_OOB;
    mbctx->pquant = pict->pquant;
    mbctx->altpquant = pict->altpquant;
    mbctx->dqedge = pict->dqedge;
    mbctx->fastuvmc = v->seq->profile == PROFILE_MAIN ? ((VC1MainSeqCtx*)v->seq)->fastuvmc : 1;
    mbctx->edge_emu_buffer = s->sc.edge_emu_buffer;
    mbctx->rnd = v->rnd;
    mbctx->ref[COMPONENT_LUMA] = s->last_picture.f->data[0];
    mbctx->ref[COMPONENT_CB] = s->last_picture.f->data[1];
    mbctx->ref[COMPONENT_CR] = s->last_picture.f->data[2];
    mbctx->use_intensity_comp = v->last_use_ic;

    intra_blkctx.btype = BLOCK_INTRA;
    intra_blkctx.vc1dsp = &v->vc1dsp;
    intra_blkctx.idsp = &s->idsp;
    intra_blkctx.s_blkctx = mbctx->s_blkctx;
    intra_blkctx.zz_8x8 = &pict->zz_8x8;
    intra_blkctx.fasttx = ((VC1SimpleSeqCtx*)v->seq)->res_fasttx;
    intra_blkctx.ac_level_code_size = &pict->ac_level_code_size;
    intra_blkctx.ac_run_code_size = &pict->ac_run_code_size;
    intra_blkctx.esc_mode3_vlc = pict->pquant < 8 || pict->dqedge;
    intra_blkctx.block = mbctx->block;
    intra_blkctx.use_cbpcy_pred = 0;
    if (!CONFIG_GRAY)
        intra_blkctx.skip_output = 0;

    inter_blkctx.btype = BLOCK_INTER;
    inter_blkctx.vc1dsp = &v->vc1dsp;
    inter_blkctx.idsp = &s->idsp;
    inter_blkctx.s_blkctx = mbctx->s_blkctx;
    inter_blkctx.block = mbctx->block;
    inter_blkctx.res_rtm_flag = v->seq->profile == PROFILE_ADVANCED ? 1 : ((VC1SimpleSeqCtx*)v->seq)->res_rtm_flag;
    inter_blkctx.ttblk_vlc = mbctx->ttblk_vlc;
    inter_blkctx.subblkpat_vlc = mbctx->subblkpat_vlc;
    if (!CONFIG_GRAY)
        inter_blkctx.skip_output = 0;

    apply_loop_filter   = s->loop_filter && !(s->avctx->skip_loop_filter >= AVDISCARD_NONKEY);
    s->first_slice_line = v->slicectx.first_line = 1;
    memset(v->cbp_base, 0, sizeof(v->cbp_base[0]) * 3 * s->mb_stride);

    i = 0;
    mbctx->mboffset_qpel[MV_Y] = s->start_mb_y * 64;

    for (s->mb_y = s->start_mb_y; s->mb_y < s->end_mb_y; s->mb_y++) {
        s->mb_x = 0;
        mbctx->mboffset_qpel[MV_X] = 0;
        init_block_index(v);

        for (; s->mb_x < s->mb_width; s->mb_x++) {
            update_block_index((VC1MBCtx*)mbctx, blkidx, s->mb_width, s->mb_width * (s->end_mb_y - s->start_mb_y), i);
            ff_update_block_index(s);

            if (v->fcm == ILACE_FIELD) {
                vc1_decode_p_mb_intfi(v);
                if (apply_loop_filter)
                    ff_vc1_p_loop_filter(v);
            } else if (v->fcm == ILACE_FRAME) {
                vc1_decode_p_mb_intfr(v);
                if (apply_loop_filter)
                    ff_vc1_p_intfr_loop_filter(v);
            } else {
                if (v->seq->profile == PROFILE_ADVANCED)
                    vc1_decode_p_mb_adv(v, mbctx, &intra_blkctx, &inter_blkctx, blkidx, gb);
                else
                    vc1_decode_p_mb(v, mbctx, &intra_blkctx, &inter_blkctx, blkidx, gb);

                topleft_mbidx = blkidx[MBIDX] < s->mb_width + 1 ? blkidx[MBIDX] + s->mb_width + 2 : blkidx[MBIDX] - s->mb_width - 1;
                toptopleft_mbidx = blkidx[MBIDX] < 2 * s->mb_width + 1 ? blkidx[MBIDX] + 2 : blkidx[MBIDX] - 2 * s->mb_width - 1;
                toptopleftleft_mbidx = blkidx[MBIDX] < 2 * s->mb_width + 2 ? blkidx[MBIDX] + 1 : blkidx[MBIDX] - 2 * s->mb_width - 2;
                curr_blkidx = blkidx[BLOCKIDX_Y0];
                topleft_blkidx = 6 * ((i + 1) % (v->end_mb_x + 2));
                left_blkidx = 6 * ((i - 1) % (v->end_mb_x + 2));

                if (v->seq->profile != PROFILE_ADVANCED) {
                    ff_vc1_h_overlap_filter((VC1MBCtx*)mbctx, curr_blkidx, left_blkidx);

                    if (i > 0)
                        ff_vc1_v_overlap_filter((VC1MBCtx*)mbctx, left_blkidx, topleft_blkidx);

                    if (i > s->mb_width) {
                        dest = mbctx->s_mbctx[topleft_mbidx].dest;

                        vc1_put_pixels((VC1MBCtx*)mbctx, topleft_blkidx, dest);

                        ff_vc1_v_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[topleft_mbidx].loopfilter_blk, dest);
                    }

                    if (i > 2 * s->mb_width) {
                        uint8_t *subblk_dest[COMPONENT_MAX] = {
                            mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_LUMA] + 4 * mbctx->linesize[COMPONENT_TYPE_LUMA],
                            mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_CB] + 4 * mbctx->linesize[COMPONENT_TYPE_CHROMA],
                            mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_CR] + 4 * mbctx->linesize[COMPONENT_TYPE_CHROMA]
                        };

                        dest = mbctx->s_mbctx[toptopleft_mbidx].dest;

                        ff_vc1_v_loop_filter((VC1MBCtx*)mbctx,
                                             mbctx->s_mbctx[toptopleft_mbidx].loopfilter_subblk,
                                             subblk_dest);
                        ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleft_mbidx].loopfilter_blk >> 2, dest);
                    }

                    if (i > 2 * s->mb_width + 1) {
                        uint8_t *subblk_dest[COMPONENT_MAX] = {
                            mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_LUMA] + 4,
                            mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_CB] + 4,
                            mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_CR] + 4
                        };

                        ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleftleft_mbidx].loopfilter_subblk >> 2, subblk_dest);
                    }

                    if (s->mb_x == s->mb_width - 1 && s->mb_y == s->end_mb_y - 1) {
                        int tail = 2 * s->mb_width + 1;

                        do {
                            i++;
                            update_block_index((VC1MBCtx*)mbctx, blkidx, s->mb_width, s->mb_width * (s->end_mb_y - s->start_mb_y), i);

                            topleft_mbidx = blkidx[MBIDX] < s->mb_width + 1 ? blkidx[MBIDX] + s->mb_width + 2 : blkidx[MBIDX] - s->mb_width - 1;
                            toptopleft_mbidx = blkidx[MBIDX] < 2 * s->mb_width + 1 ? blkidx[MBIDX] + 2 : blkidx[MBIDX] - 2 * s->mb_width - 1;
                            toptopleftleft_mbidx = blkidx[MBIDX] < 2 * s->mb_width + 2 ? blkidx[MBIDX] + 1 : blkidx[MBIDX] - 2 * s->mb_width - 2;
                            curr_blkidx = blkidx[BLOCKIDX_Y0];
                            topleft_blkidx = 6 * ((i + 1) % (v->end_mb_x + 2));
                            left_blkidx = 6 * ((i - 1) % (v->end_mb_x + 2));

                            if (tail == 2 * s->mb_width + 1)
                                ff_vc1_v_overlap_filter((VC1MBCtx*)mbctx, left_blkidx, topleft_blkidx);

                            if (tail > s->mb_width) {
                                dest = mbctx->s_mbctx[topleft_mbidx].dest;

                                vc1_put_pixels((VC1MBCtx*)mbctx, topleft_blkidx, dest);

                                ff_vc1_v_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[topleft_mbidx].loopfilter_blk, dest);
                            }

                            if (tail > 0) {
                                uint8_t *subblk_dest[COMPONENT_MAX] = {
                                    mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_LUMA] + 4 * mbctx->linesize[COMPONENT_TYPE_LUMA],
                                    mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_CB] + 4 * mbctx->linesize[COMPONENT_TYPE_CHROMA],
                                    mbctx->s_mbctx[toptopleft_mbidx].dest[COMPONENT_CR] + 4 * mbctx->linesize[COMPONENT_TYPE_CHROMA]
                                };

                                dest = mbctx->s_mbctx[toptopleft_mbidx].dest;

                                ff_vc1_v_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleft_mbidx].loopfilter_subblk, subblk_dest);
                                ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleft_mbidx].loopfilter_blk >> 2, dest);
                            }

                            {
                                uint8_t *subblk_dest[COMPONENT_MAX] = {
                                    mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_LUMA] + 4,
                                    mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_CB] + 4,
                                    mbctx->s_mbctx[toptopleftleft_mbidx].dest[COMPONENT_CR] + 4
                                };

                                ff_vc1_h_loop_filter((VC1MBCtx*)mbctx, mbctx->s_mbctx[toptopleftleft_mbidx].loopfilter_subblk >> 2, subblk_dest);
                            }
                        } while (tail--);
                    }
                }

                if (v->seq->profile == PROFILE_ADVANCED && apply_loop_filter)
                    ff_vc1_p_loop_filter(v);
            }
            if (get_bits_left(&s->gb) < 0 || get_bits_count(&s->gb) < 0) {
                // TODO: may need modification to handle slice coding
                ff_er_add_slice(&s->er, 0, s->start_mb_y, s->mb_x, s->mb_y, ER_MB_ERROR);
                av_log(s->avctx, AV_LOG_ERROR, "Bits overconsumption: %i > %i at %ix%i\n",
                       get_bits_count(&s->gb), s->gb.size_in_bits, s->mb_x, s->mb_y);
                return;
            }
            inc_blk_idx(v->topleft_blk_idx);
            inc_blk_idx(v->top_blk_idx);
            inc_blk_idx(v->left_blk_idx);
            inc_blk_idx(v->cur_blk_idx);

            i++;
            mbctx->mboffset_qpel[MV_X] += 64;
        }

        memmove(v->cbp_base,
                v->cbp - s->mb_stride,
                sizeof(v->cbp_base[0]) * 2 * s->mb_stride);
        memmove(v->ttblk_base,
                v->ttblk - s->mb_stride,
                sizeof(v->ttblk_base[0]) * 2 * s->mb_stride);
        memmove(v->is_intra_base,
                v->is_intra - s->mb_stride,
                sizeof(v->is_intra_base[0]) * 2 * s->mb_stride);
        memmove(v->luma_mv_base,
                v->luma_mv - s->mb_stride,
                sizeof(v->luma_mv_base[0]) * 2 * s->mb_stride);
        if (s->mb_y != s->start_mb_y)
            ff_mpeg_draw_horiz_band(s, (s->mb_y - 1) * 16, 16);
        s->first_slice_line = v->slicectx.first_line = 0;

        mbctx->mboffset_qpel[MV_Y] += 64;
    }
    if (s->end_mb_y >= s->start_mb_y)
        ff_mpeg_draw_horiz_band(s, (s->end_mb_y - 1) * 16, 16);
    ff_er_add_slice(&s->er, 0, s->start_mb_y << v->field_mode, s->mb_width - 1,
                    (s->end_mb_y << v->field_mode) - 1, ER_MB_END);
}

static void vc1_decode_b_blocks(VC1Context *v)
{
    MpegEncContext *s = &v->s;

    /* select coding mode used for VLC tables selection */
    switch (v->c_ac_table_index) {
    case 0:
        v->codingset = (v->pqindex <= 8) ? CS_HIGH_RATE_INTRA : CS_LOW_MOT_INTRA;
        break;
    case 1:
        v->codingset = CS_HIGH_MOT_INTRA;
        break;
    case 2:
        v->codingset = CS_MID_RATE_INTRA;
        break;
    }

    switch (v->c_ac_table_index) {
    case 0:
        v->codingset2 = (v->pqindex <= 8) ? CS_HIGH_RATE_INTER : CS_LOW_MOT_INTER;
        break;
    case 1:
        v->codingset2 = CS_HIGH_MOT_INTER;
        break;
    case 2:
        v->codingset2 = CS_MID_RATE_INTER;
        break;
    }

    s->first_slice_line = 1;
    for (s->mb_y = s->start_mb_y; s->mb_y < s->end_mb_y; s->mb_y++) {
        s->mb_x = 0;
        init_block_index(v);
        for (; s->mb_x < s->mb_width; s->mb_x++) {
            ff_update_block_index(s);

            if (v->fcm == ILACE_FIELD) {
                vc1_decode_b_mb_intfi(v);
                if (v->s.loop_filter)
                    ff_vc1_b_intfi_loop_filter(v);
            } else if (v->fcm == ILACE_FRAME) {
                vc1_decode_b_mb_intfr(v);
                if (v->s.loop_filter)
                    ff_vc1_p_intfr_loop_filter(v);
            } else {
                vc1_decode_b_mb(v);
                if (v->s.loop_filter)
                    ff_vc1_i_loop_filter(v);
            }
            if (get_bits_left(&s->gb) < 0 || get_bits_count(&s->gb) < 0) {
                // TODO: may need modification to handle slice coding
                ff_er_add_slice(&s->er, 0, s->start_mb_y, s->mb_x, s->mb_y, ER_MB_ERROR);
                av_log(s->avctx, AV_LOG_ERROR, "Bits overconsumption: %i > %i at %ix%i\n",
                       get_bits_count(&s->gb), s->gb.size_in_bits, s->mb_x, s->mb_y);
                return;
            }
        }
        memmove(v->cbp_base,
                v->cbp - s->mb_stride,
                sizeof(v->cbp_base[0]) * 2 * s->mb_stride);
        memmove(v->ttblk_base,
                v->ttblk - s->mb_stride,
                sizeof(v->ttblk_base[0]) * 2 * s->mb_stride);
        memmove(v->is_intra_base,
                v->is_intra - s->mb_stride,
                sizeof(v->is_intra_base[0]) * 2 * s->mb_stride);
        if (!v->s.loop_filter)
            ff_mpeg_draw_horiz_band(s, s->mb_y * 16, 16);
        else if (s->mb_y)
            ff_mpeg_draw_horiz_band(s, (s->mb_y - 1) * 16, 16);
        s->first_slice_line = 0;
    }
    if (v->s.loop_filter)
        ff_mpeg_draw_horiz_band(s, (s->end_mb_y - 1) * 16, 16);
    ff_er_add_slice(&s->er, 0, s->start_mb_y << v->field_mode, s->mb_width - 1,
                    (s->end_mb_y << v->field_mode) - 1, ER_MB_END);
}

static void vc1_decode_skip_blocks(VC1Context *v)
{
    MpegEncContext *s = &v->s;

    if (!v->s.last_picture.f->data[0])
        return;

    ff_er_add_slice(&s->er, 0, s->start_mb_y, s->mb_width - 1, s->end_mb_y - 1, ER_MB_END);
    s->first_slice_line = 1;
    for (s->mb_y = s->start_mb_y; s->mb_y < s->end_mb_y; s->mb_y++) {
        s->mb_x = 0;
        init_block_index(v);
        ff_update_block_index(s);
        memcpy(s->dest[0], s->last_picture.f->data[0] + s->mb_y * 16 * s->linesize,   s->linesize   * 16);
        memcpy(s->dest[1], s->last_picture.f->data[1] + s->mb_y *  8 * s->uvlinesize, s->uvlinesize *  8);
        memcpy(s->dest[2], s->last_picture.f->data[2] + s->mb_y *  8 * s->uvlinesize, s->uvlinesize *  8);
        ff_mpeg_draw_horiz_band(s, s->mb_y * 16, 16);
        s->first_slice_line = 0;
    }
    s->pict_type = AV_PICTURE_TYPE_P;
}

void ff_vc1_decode_blocks(VC1Context *v)
{
    VC1SeqCtx *seq = v->seq;

    v->s.esc3_level_length = 0;
    if (v->x8_type) {
        ff_intrax8_decode_picture(&v->x8, &v->s.current_picture,
                                  &v->s.gb, &v->s.mb_x, &v->s.mb_y,
                                  2 * v->pq + v->halfpq, v->pq * !v->pquantizer,
                                  v->s.loop_filter, v->s.low_delay);

        ff_er_add_slice(&v->s.er, 0, 0,
                        (v->s.mb_x >> 1) - 1, (v->s.mb_y >> 1) - 1,
                        ER_MB_END);
    } else {
        v->cur_blk_idx     =  0;
        v->left_blk_idx    = -1;
        v->topleft_blk_idx =  1;
        v->top_blk_idx     =  2;
        switch (v->s.pict_type) {
        case AV_PICTURE_TYPE_I:
            if (seq->profile == PROFILE_ADVANCED)
                vc1_decode_i_blocks_adv(v);
            else
                vc1_decode_i_blocks(v);
            break;
        case AV_PICTURE_TYPE_P:
            if (v->p_frame_skipped)
                vc1_decode_skip_blocks(v);
            else
                vc1_decode_p_blocks(v);
            break;
        case AV_PICTURE_TYPE_B:
            if (v->bi_type) {
                if (seq->profile == PROFILE_ADVANCED)
                    vc1_decode_i_blocks_adv(v);
                else
                    vc1_decode_i_blocks(v);
            } else
                vc1_decode_b_blocks(v);
            break;
        }
    }
}
