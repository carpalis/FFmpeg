/*
 * VC-1 and WMV3 decoder
 * copyright (c) 2006 Konstantin Shishkov
 * (c) 2005 anonymous, Alex Beregszaszi, Michael Niedermayer
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
 * VC-1 tables.
 */

#ifndef AVCODEC_VC1DATA_H
#define AVCODEC_VC1DATA_H

#include <stdint.h>

#include "libavutil/rational.h"

#include "vlc.h"
#include "vc1.h"

/** Table for conversion between TTBLK and TTMB */
extern const int ff_vc1_ttblk_to_tt[3][8];

extern const int ff_vc1_ttfrm_to_tt[4];

/** MV P mode - the 5th element is only used for mode 1 */
extern const uint8_t ff_vc1_mv_pmode_table[2][5];
extern const uint8_t ff_vc1_mv_pmode_table2[2][4];

/* MBMODE table for interlaced frame P-picture */
extern const uint8_t ff_vc1_mbmode_intfrp[2][15][4];

/** @name VC-1 VLC tables and defines
 *  @todo TODO move this into the context
 */
//@{
#define VC1_BFRACTION_VLC_BITS 7
extern VLC ff_vc1_bfraction_vlc;
#define VC1_IMODE_VLC_BITS 4
extern VLC ff_vc1_imode_vlc;
#define VC1_NORM2_VLC_BITS 3
extern VLC ff_vc1_norm2_vlc;
#define VC1_NORM6_VLC_BITS 9
extern VLC ff_vc1_norm6_vlc;
/* Could be optimized, one table only needs 8 bits */
#define VC1_TTMB_VLC_BITS 9 //12
extern VLC ff_vc1_ttmb_vlc[3];
#define VC1_MV_DIFF_VLC_BITS 9 //15
extern VLC ff_vc1_mv_diff_vlc[4];
#define VC1_CBPCY_P_VLC_BITS 9 //14
extern VLC ff_vc1_cbpcy_p_vlc[4];
#define VC1_ICBPCY_VLC_BITS 9
extern VLC ff_vc1_icbpcy_vlc[8];
#define VC1_4MV_BLOCK_PATTERN_VLC_BITS 6
extern VLC ff_vc1_4mv_block_pattern_vlc[4];
#define VC1_2MV_BLOCK_PATTERN_VLC_BITS 3
extern VLC ff_vc1_2mv_block_pattern_vlc[4];
#define VC1_TTBLK_VLC_BITS 5
extern VLC ff_vc1_ttblk_vlc[3];
#define VC1_SUBBLKPAT_VLC_BITS 6
extern VLC ff_vc1_subblkpat_vlc[3];
#define VC1_INTFR_4MV_MBMODE_VLC_BITS 9
extern VLC ff_vc1_intfr_4mv_mbmode_vlc[4];
#define VC1_INTFR_NON4MV_MBMODE_VLC_BITS 6
extern VLC ff_vc1_intfr_non4mv_mbmode_vlc[4];
#define VC1_IF_MMV_MBMODE_VLC_BITS 5
extern VLC ff_vc1_if_mmv_mbmode_vlc[8];
#define VC1_IF_1MV_MBMODE_VLC_BITS 5
extern VLC ff_vc1_if_1mv_mbmode_vlc[8];
#define VC1_1REF_MVDATA_VLC_BITS 9
extern VLC ff_vc1_1ref_mvdata_vlc[4];
#define VC1_2REF_MVDATA_VLC_BITS 9
extern VLC ff_vc1_2ref_mvdata_vlc[8];

extern VLC ff_vc1_ac_coeff_table[8];

#define VC1_IF_MBMODE_VLC_BITS 5
//@}

#define B_FRACTION_DEN  256

/* pre-computed scales for all bfractions and base=256 */
extern const int16_t ff_vc1_bfraction_lut[23];
extern const uint8_t ff_vc1_bfraction_bits[23];
extern const uint8_t ff_vc1_bfraction_codes[23];

/* BitPlane IMODE - such a small table... */
extern const uint8_t ff_vc1_imode_codes[7];
extern const uint8_t ff_vc1_imode_bits[7];

/* Normal-2 imode */
extern const uint8_t ff_vc1_norm2_codes[4];
extern const uint8_t ff_vc1_norm2_bits[4];
extern const uint16_t ff_vc1_norm6_codes[64];
extern const uint8_t ff_vc1_norm6_bits[64];
/* Normal-6 imode */
extern const uint8_t ff_vc1_norm6_spec[64][5];

/* 4MV Block pattern VLC tables */
extern const uint8_t ff_vc1_4mv_block_pattern_codes[4][16];
extern const uint8_t ff_vc1_4mv_block_pattern_bits[4][16];

/* 2MV Block pattern VLC tables */
extern const uint8_t ff_vc1_2mv_block_pattern_codes[4][4];
extern const uint8_t ff_vc1_2mv_block_pattern_bits[4][4];

extern const uint8_t ff_wmv3_dc_scale_table[32];

/* P-Picture CBPCY VLC tables */
extern const uint16_t ff_vc1_cbpcy_p_codes[4][64];
extern const uint8_t ff_vc1_cbpcy_p_bits[4][64];

/* Interlaced CBPCY VLC tables (Table 124 - Table 131) */
extern const uint16_t ff_vc1_icbpcy_p_codes[8][63];
extern const uint8_t ff_vc1_icbpcy_p_bits[8][63];

/* MacroBlock Transform Type: 7.1.3.11, p89
 * 8x8:B
 * 8x4:B:btm  8x4:B:top  8x4:B:both,
 * 4x8:B:right  4x8:B:left  4x8:B:both
 * 4x4:B  8x8:MB
 * 8x4:MB:btm  8x4:MB:top  8x4,MB,both
 * 4x8,MB,right  4x8,MB,left
 * 4x4,MB                               */
extern const uint16_t ff_vc1_ttmb_codes[3][16];

extern const uint8_t ff_vc1_ttmb_bits[3][16];

/* TTBLK (Transform Type per Block) tables */
extern const uint8_t ff_vc1_ttblk_codes[3][8];
extern const uint8_t ff_vc1_ttblk_bits[3][8];

/* SUBBLKPAT tables, p93-94, reordered */
extern const uint8_t ff_vc1_subblkpat_codes[3][15];
extern const uint8_t ff_vc1_subblkpat_bits[3][15];

/* MV differential tables, p265 */
extern const uint16_t ff_vc1_mv_diff_codes[4][73];
extern const uint8_t ff_vc1_mv_diff_bits[4][73];

/* Interlaced frame picture MBMODE VLC tables (p. 246, p. 360) */
extern const uint16_t ff_vc1_intfr_4mv_mbmode_codes[4][15];
extern const uint8_t ff_vc1_intfr_4mv_mbmode_bits[4][15];
extern const uint8_t ff_vc1_intfr_non4mv_mbmode_codes[4][9];
extern const uint8_t ff_vc1_intfr_non4mv_mbmode_bits[4][9];

/* Interlaced field picture MBMODE VLC tables (p. 356 - 11.4.1, 11.4.2) */
extern const uint8_t ff_vc1_if_mmv_mbmode_codes[8][8];
extern const uint8_t ff_vc1_if_mmv_mbmode_bits[8][8];
extern const uint8_t ff_vc1_if_1mv_mbmode_codes[8][6];
extern const uint8_t ff_vc1_if_1mv_mbmode_bits[8][6];

/* Interlaced frame/field picture MVDATA VLC tables */
/* 1-reference tables */
extern const uint32_t ff_vc1_1ref_mvdata_codes[4][72];
extern const uint8_t ff_vc1_1ref_mvdata_bits[4][72];
/* 2-reference tables */
extern const uint32_t ff_vc1_2ref_mvdata_codes[8][126];
extern const uint8_t ff_vc1_2ref_mvdata_bits[8][126];

/* DC differentials low+hi-mo, p217 are the same as in msmpeg4data .h */

/* Scantables/ZZ scan are at 11.9 (p262) and 8.1.1.12 (p10) */
extern const uint8_t ff_vc1_simple_progressive_4x4_zz [16];
extern const uint8_t ff_vc1_adv_progressive_8x4_zz [32];
extern const uint8_t ff_vc1_adv_progressive_4x8_zz [32];
extern const uint8_t ff_vc1_adv_interlaced_8x8_zz [64];
extern const uint8_t ff_vc1_adv_interlaced_8x4_zz [32];
extern const uint8_t ff_vc1_adv_interlaced_4x8_zz [32];
extern const uint8_t ff_vc1_adv_interlaced_4x4_zz [16];
extern const uint8_t ff_vc1_intra_horz_8x8_zz [64];
extern const uint8_t ff_vc1_intra_vert_8x8_zz [64];

/* ASPECT_RATIO to Sample Aspect Ratio (Table 7) as specified in 6.1.14.3.1 */
extern const AVRational ff_vc1_sample_aspect_ratio[14];

/* FRAMERATENR to Frame Rate Numerator (Table 8) as specified in 6.1.14.4.2 */
extern const uint8_t ff_vc1_fps_num[7];

/* FRAMERATEDR to Frame Rate Denominator (Table 9) as specified in 6.1.14.4.3 */
extern const uint16_t ff_vc1_fps_den[2];

/* PQINDEX to PQUANT (Table 36) as specified in 7.1.1.6 */
extern const uint8_t ff_vc1_pquant_table[32];

/* MVMODE (Tables 46 & 47) as specified in 7.1.1.32 */
extern const uint8_t ff_vc1_mvmode_table[2][5];

/* MVMODE2 (Tables 49 & 50) as specified in 7.1.1.33 */
extern const uint8_t ff_vc1_mvmode2_table[2][4];

/* MQUANT to DCStepSize as specified in 8.1.3.3 */
extern const int8_t ff_vc1_dc_scale_table[32];

/* AC Coding Sets (Tables 71 & 72) as specified in 8.1.3.4
 * VC1ACCodingSet ff_vc1_ac_coding_set[CodingSet][PictureComponent]
 */
extern const VC1ACCodingSet ff_vc1_ac_coding_set[CS_MAX][COMPONENT_MAX];

/* DQScale (Table 74) as specified in 8.1.3.9 */
extern const int32_t ff_vc1_dqscale[64];

/* I-Picture CBPCY VLC Table (Table 168)
 * as specified in 11.5
 */
extern const uint16_t ff_vc1_i_cbpcy_codes[64];
extern const uint8_t ff_vc1_i_cbpcy_bits[64];

/* P and B-Picture CBPCY VLC Table 0 (Table 169)
 * as specified in 11.6
 */
extern const uint8_t ff_vc1_p_cbpcy_0_codes[64];
extern const uint8_t ff_vc1_p_cbpcy_0_bits[64];

/* P and B-Picture CBPCY VLC Table 1 (Table 170)
 * as specified in 11.6
 */
extern const uint8_t ff_vc1_p_cbpcy_1_codes[64];
extern const uint8_t ff_vc1_p_cbpcy_1_bits[64];

/* P and B-Picture CBPCY VLC Table 2 (Table 171)
 * as specified in 11.6
 */
extern const uint16_t ff_vc1_p_cbpcy_2_codes[64];
extern const uint8_t ff_vc1_p_cbpcy_2_bits[64];

/* P and B-Picture CBPCY VLC Table 3 (Table 172)
 * as specified in 11.6
 */
extern const uint8_t ff_vc1_p_cbpcy_3_codes[64];
extern const uint8_t ff_vc1_p_cbpcy_3_bits[64];

/* Low-motion Luma DC Differential VLC Table (Table 173)
 * as specified in 11.7.1$
 */
extern const uint32_t ff_vc1_low_motion_luma_dc_codes[120];
extern const uint8_t ff_vc1_low_motion_luma_dc_bits[120];

/* Low-motion Color-difference DC Differential VLC Table (Table 174)
 * as specified in 11.7.1$
 */
extern const uint32_t ff_vc1_low_motion_chroma_dc_codes[120];
extern const uint8_t ff_vc1_low_motion_chroma_dc_bits[120];

/* High-motion Luma DC Differential VLC Table (Table 175)
 * as specified in 11.7.2$
 */
extern const uint32_t ff_vc1_high_motion_luma_dc_codes[120];
extern const uint8_t ff_vc1_high_motion_luma_dc_bits[120];

/* High-motion Color-difference DC Differential VLC Table (Table 176)
 * as specified in 11.7.2$
 */
extern const uint32_t ff_vc1_high_motion_chroma_dc_codes[120];
extern const uint8_t ff_vc1_high_motion_chroma_dc_bits[120];

/* High-motion Intra VLC Table (Table 177)
 * as specified in 11.8.1
 */
extern const uint16_t ff_vc1_high_motion_intra_index_codes[188];
extern const uint8_t ff_vc1_high_motion_intra_index_bits[188];
extern const uint16_t ff_vc1_high_motion_intra_index_symbols[188];

/* High-motion Inter VLC Table (Table 184)
 * as specified in 11.8.1
 */
extern const uint16_t ff_vc1_high_motion_inter_index_codes[171];
extern const uint8_t ff_vc1_high_motion_inter_index_bits[171];
extern const uint16_t ff_vc1_high_motion_inter_index_symbols[171];

/* Low-motion Intra VLC Table (Table 191)
 * as specified in 11.8.2
 */
extern const uint16_t ff_vc1_low_motion_intra_index_codes[135];
extern const uint8_t ff_vc1_low_motion_intra_index_bits[135];
extern const uint16_t ff_vc1_low_motion_intra_index_symbols[135];

/* Low-motion Inter VLC Table (Table 198)
 * as specified in 11.8.3
 */
extern const uint16_t ff_vc1_low_motion_inter_index_codes[151];
extern const uint8_t ff_vc1_low_motion_inter_index_bits[151];
extern const uint16_t ff_vc1_low_motion_inter_index_symbols[151];

/* Mid Rate Intra VLC Table (Table 205)
 * as specified in 11.8.4
 */
extern const uint8_t ff_vc1_mid_rate_intra_index_codes[105];
extern const uint8_t ff_vc1_mid_rate_intra_index_bits[105];
extern const uint16_t ff_vc1_mid_rate_intra_index_symbols[105];

/* Mid Rate Inter VLC Table (Table 212)
 * as specified in 11.8.5
 */
extern const uint8_t ff_vc1_mid_rate_inter_index_codes[105];
extern const uint8_t ff_vc1_mid_rate_inter_index_bits[105];
extern const uint16_t ff_vc1_mid_rate_inter_index_symbols[105];

/* High Rate Intra VLC Table (Table 219)
 * as specified in 11.8.6
 */
extern const uint16_t ff_vc1_high_rate_intra_index_codes[165];
extern const uint8_t ff_vc1_high_rate_intra_index_bits[165];
extern const uint16_t ff_vc1_high_rate_intra_index_symbols[165];

/* High Rate Inter VLC Table (Table 226)
 * as specified in 11.8.7
 */
extern const uint32_t ff_vc1_high_rate_inter_index_codes[177];
extern const uint8_t ff_vc1_high_rate_inter_index_bits[177];
extern const uint16_t ff_vc1_high_rate_inter_index_symbols[177];

/* Intra Normal Scan Zigzag Table (Table 233)
 * as specified in 11.9.1 (transposed)
 */
extern const int8_t ff_vc1_intra_8x8_normal_scan_zz_table[64];

/* Intra Horizontal Scan Zigzag Table (Table 234)
 * as specified in 11.9.1 (transposed)
 */
extern const int8_t ff_vc1_intra_8x8_horiz_scan_zz_table[64];

/* Intra Vertical Scan Zigzag Table (Table 235)
 * as specified in 11.9.1 (transposed)
 */
extern const int8_t ff_vc1_intra_8x8_vert_scan_zz_table[64];

/* Inter 8x8 Scan Zigzag Table (Table 236)
 * as specified in 11.9.2 (transposed)
 */
extern const int8_t ff_vc1_inter_8x8_scan_zz_table[64];

/* Inter 8x4 Scan Zigzag Table (Table 237)
 * as specified in 11.9.2
 */
extern const int8_t ff_vc1_inter_8x4_scan_zz_table[32];

/* Inter 4x8 Scan Zigzag Table (Table 238)
 * as specified in 11.9.2 (adapted)
 */
extern const int8_t ff_vc1_inter_4x8_scan_zz_table[32];

/* Inter 4x4 Scan Zigzag Table (Table 239)
 * as specified in 11.9.2 (adapted)
 */
extern const int8_t ff_vc1_inter_4x4_scan_zz_table[16];

/* P Interlaced field picture MV predictor scaling values (Table 114) */
extern const uint16_t ff_vc1_field_mvpred_scales[2][7][4];
/* B Interlaced field picture backward MV predictor scaling values for first field (Table 115) */
extern const uint16_t ff_vc1_b_field_mvpred_scales[7][4];

#define AC_MODES 8

extern const int ff_vc1_ac_sizes[AC_MODES];

#endif /* AVCODEC_VC1DATA_H */
