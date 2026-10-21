// Minimal baseline JPEG encoder for NV12 input (header-only, no dependencies).
//
// Baseline sequential DCT, Huffman tables from ITU T.81 Annex K, IJG quality
// scaling, 4:2:0 (16x16 MCU: 4 Y + Cb + Cr). NV12 full-range BT.601 is
// exactly JFIF YCbCr, so the planes are encoded as-is. Float AAN forward DCT
// with quantisation folded into the output scaling (as in IJG jfdctflt.c).
// Edges are padded by replicating the last row/column.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace roadsense {

class JpegNV12Encoder {
public:
    explicit JpegNV12Encoder(int quality = 80) { set_quality(quality); build_huffman(); }

    void set_quality(int q) {
        static const uint8_t lq[64] = {16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
                                       14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
                                       18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
                                       49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};
        static const uint8_t cq[64] = {17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
                                       24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
                                       99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
                                       99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};
        static const float aan[8] = {1.0f, 1.387039845f, 1.306562965f, 1.175875602f,
                                     1.0f, 0.785694958f, 0.541196100f, 0.275899379f};
        q = q < 1 ? 1 : (q > 100 ? 100 : q);
        const int scale = q < 50 ? 5000 / q : 200 - 2 * q;
        for (int i = 0; i < 64; ++i) {
            int l = (lq[i] * scale + 50) / 100, c = (cq[i] * scale + 50) / 100;
            qt_[0][i] = uint8_t(l < 1 ? 1 : (l > 255 ? 255 : l));
            qt_[1][i] = uint8_t(c < 1 ? 1 : (c > 255 ? 255 : c));
        }
        for (int r = 0; r < 8; ++r)
            for (int c = 0; c < 8; ++c) {
                const int i = r * 8 + c;
                fq_[0][i] = 1.0f / (qt_[0][i] * aan[r] * aan[c] * 8.0f);
                fq_[1][i] = 1.0f / (qt_[1][i] * aan[r] * aan[c] * 8.0f);
            }
        for (int r = 0; r < 8; ++r)
            for (int c = 0; c < 8; ++c)
                for (int t = 0; t < 2; ++t) fqt_[t][c * 8 + r] = fq_[t][r * 8 + c];
        for (int k = 0; k < 64; ++k) zzt_[k] = uint8_t((zigzag()[k] % 8) * 8 + zigzag()[k] / 8);
    }

    // Encodes width x height NV12 (Y rows y_stride bytes, interleaved CbCr rows
    // uv_stride bytes). Width and height must be even. Returns the JPEG bytes.
    const std::vector<uint8_t> &encode(const uint8_t *y, size_t y_stride, const uint8_t *uv, size_t uv_stride,
                                       int width, int height) {
        out_.clear();
        out_.reserve(size_t(width) * height / 4 + 1024);
        write_headers(width, height);
        bitbuf_ = 0;
        bitcnt_ = 0;
        int dc[3] = {0, 0, 0};
        const int cw = width / 2, chh = height / 2;
        alignas(16) float blk[64];
        for (int my = 0; my < height; my += 16) {
            for (int mx = 0; mx < width; mx += 16) {
                for (int b = 0; b < 4; ++b) {
                    const int bx = mx + (b & 1) * 8, by = my + (b >> 1) * 8;
                    for (int r = 0; r < 8; ++r) {
                        const int yy = by + r < height ? by + r : height - 1;
                        const uint8_t *row = y + size_t(yy) * y_stride;
                        if (bx + 8 <= width) {
                            for (int c = 0; c < 8; ++c) blk[r * 8 + c] = float(row[bx + c]) - 128.0f;
                        } else {
                            for (int c = 0; c < 8; ++c) {
                                const int xx = bx + c < width ? bx + c : width - 1;
                                blk[r * 8 + c] = float(row[xx]) - 128.0f;
                            }
                        }
                    }
                    dc[0] = encode_block(blk, 0, dc[0]);
                }
                const int cx0 = mx / 2, cy0 = my / 2;
                for (int comp = 0; comp < 2; ++comp) {
                    for (int r = 0; r < 8; ++r) {
                        const int yy = cy0 + r < chh ? cy0 + r : chh - 1;
                        const uint8_t *row = uv + size_t(yy) * uv_stride;
                        for (int c = 0; c < 8; ++c) {
                            const int xx = cx0 + c < cw ? cx0 + c : cw - 1;
                            blk[r * 8 + c] = float(row[2 * xx + comp]) - 128.0f;
                        }
                    }
                    dc[1 + comp] = encode_block(blk, 1, dc[1 + comp]);
                }
            }
        }
        // pad the last byte with 1-bits
        const int pad = (8 - (bitcnt_ & 7)) & 7;
        if (pad) put_bits(0x7F, pad);
        flush_bytes();
        out_.push_back(0xFF);
        out_.push_back(0xD9);
        return out_;
    }

private:
    struct Code {
        uint16_t code;
        uint8_t len;
    };

    static const uint8_t *zigzag() {
        static const uint8_t z[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                                      12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                                      35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                                      58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};
        return z;
    }

    // Annex K tables: [0]=DC luma, [1]=AC luma, [2]=DC chroma, [3]=AC chroma
    static const uint8_t *hbits(int t) {
        static const uint8_t b[4][16] = {{0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
                                         {0, 2, 1, 3, 3, 2, 4, 3, 5, 5, 4, 4, 0, 0, 1, 0x7d},
                                         {0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0},
                                         {0, 2, 1, 2, 4, 4, 3, 4, 7, 5, 4, 4, 0, 1, 2, 0x77}};
        return b[t];
    }
    static const uint8_t *hvals(int t) {
        static const uint8_t dc[12] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
        static const uint8_t acl[162] = {
            0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12, 0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22,
            0x71, 0x14, 0x32, 0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0, 0x24, 0x33,
            0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34,
            0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55,
            0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75, 0x76,
            0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
            0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3, 0xb4, 0xb5,
            0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4,
            0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea, 0xf1,
            0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
        static const uint8_t acc[162] = {
            0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13,
            0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0, 0x15, 0x62,
            0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17, 0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29,
            0x2a, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54,
            0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a, 0x73, 0x74, 0x75,
            0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94,
            0x95, 0x96, 0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2, 0xb3,
            0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2,
            0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
            0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
        return t == 1 ? acl : (t == 3 ? acc : dc);
    }
    static int hcount(int t) {
        int n = 0;
        for (int i = 0; i < 16; ++i) n += hbits(t)[i];
        return n;
    }

    void build_huffman() {
        for (int t = 0; t < 4; ++t) {
            std::memset(codes_[t], 0, sizeof(codes_[t]));
            const uint8_t *bits = hbits(t), *vals = hvals(t);
            int code = 0, k = 0;
            for (int len = 1; len <= 16; ++len) {
                for (int i = 0; i < bits[len - 1]; ++i) codes_[t][vals[k++]] = Code{uint16_t(code++), uint8_t(len)};
                code <<= 1;
            }
        }
    }

    void w8(int v) { out_.push_back(uint8_t(v)); }
    void w16(int v) {
        w8(v >> 8);
        w8(v & 0xFF);
    }

    void write_headers(int width, int height) {
        w16(0xFFD8);
        static const uint8_t app0[] = {0xFF, 0xE0, 0, 16, 'J', 'F', 'I', 'F', 0, 1, 1, 0, 0, 1, 0, 1, 0, 0};
        out_.insert(out_.end(), app0, app0 + sizeof(app0));
        w16(0xFFDB);
        w16(2 + 2 * 65);
        for (int t = 0; t < 2; ++t) {
            w8(t);
            for (int i = 0; i < 64; ++i) w8(qt_[t][zigzag()[i]]);
        }
        w16(0xFFC0);
        w16(17);
        w8(8);
        w16(height);
        w16(width);
        w8(3);
        const uint8_t comps[9] = {1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1};
        out_.insert(out_.end(), comps, comps + 9);
        w16(0xFFC4);
        int len = 2;
        for (int t = 0; t < 4; ++t) len += 17 + hcount(t);
        w16(len);
        static const uint8_t cls[4] = {0x00, 0x10, 0x01, 0x11};
        for (int t = 0; t < 4; ++t) {
            w8(cls[t]);
            out_.insert(out_.end(), hbits(t), hbits(t) + 16);
            out_.insert(out_.end(), hvals(t), hvals(t) + hcount(t));
        }
        static const uint8_t sos[] = {0xFF, 0xDA, 0, 12, 3, 1, 0x00, 2, 0x11, 3, 0x11, 0, 63, 0};
        out_.insert(out_.end(), sos, sos + sizeof(sos));
    }

    inline void put_bits(uint32_t bits, int n) {
        bitbuf_ = (bitbuf_ << n) | (bits & ((1u << n) - 1));
        bitcnt_ += n;
        if (bitcnt_ >= 32) flush_bytes();
    }
    inline void flush_bytes() {
        while (bitcnt_ >= 8) {
            const uint8_t b = uint8_t(bitbuf_ >> (bitcnt_ - 8));
            out_.push_back(b);
            if (b == 0xFF) out_.push_back(0);
            bitcnt_ -= 8;
        }
    }

    // AAN 1-D DCT down all 8 columns at once (the loop over c vectorises).
    static inline void fdct_cols(float *__restrict d) {
        for (int c = 0; c < 8; ++c) {
            const float t0 = d[c] + d[56 + c], t7 = d[c] - d[56 + c];
            const float t1 = d[8 + c] + d[48 + c], t6 = d[8 + c] - d[48 + c];
            const float t2 = d[16 + c] + d[40 + c], t5 = d[16 + c] - d[40 + c];
            const float t3 = d[24 + c] + d[32 + c], t4 = d[24 + c] - d[32 + c];
            const float t10 = t0 + t3, t13 = t0 - t3, t11 = t1 + t2, t12 = t1 - t2;
            d[c] = t10 + t11;
            d[32 + c] = t10 - t11;
            const float z1 = (t12 + t13) * 0.707106781f;
            d[16 + c] = t13 + z1;
            d[48 + c] = t13 - z1;
            const float o10 = t4 + t5, o11 = t5 + t6, o12 = t6 + t7;
            const float z5 = (o10 - o12) * 0.382683433f;
            const float z2 = 0.541196100f * o10 + z5;
            const float z4 = 1.306562965f * o12 + z5;
            const float z3 = o11 * 0.707106781f;
            const float z11 = t7 + z3, z13 = t7 - z3;
            d[40 + c] = z13 + z2;
            d[24 + c] = z13 - z2;
            d[8 + c] = z11 + z4;
            d[56 + c] = z11 - z4;
        }
    }
    static inline void transpose8(const float *__restrict in, float *__restrict out) {
        for (int r = 0; r < 8; ++r)
            for (int c = 0; c < 8; ++c) out[c * 8 + r] = in[r * 8 + c];
    }

    static inline int nbits(int v) {
        v = v < 0 ? -v : v;
        return v ? 32 - __builtin_clz(unsigned(v)) : 0;
    }

    // Column pass, transpose, column pass: the result is the TRANSPOSED
    // coefficient block, so the quantiser (fqt_) and zigzag (zzt_) tables
    // are stored transposed too.
    int encode_block(float *blk, int t, int prev_dc) {
        alignas(16) float tr[64];
        fdct_cols(blk);
        transpose8(blk, tr);
        fdct_cols(tr);
        int q[64];
        const float *fq = fqt_[t];
        for (int i = 0; i < 64; ++i) {
            const float v = tr[i] * fq[i];
            q[i] = int(v + (v >= 0 ? 0.5f : -0.5f));
        }
        const uint8_t *zz = zzt_;
        const Code *dct = codes_[t * 2], *act = codes_[t * 2 + 1];
        const int dcv = q[0];
        int diff = dcv - prev_dc;
        int n = nbits(diff);
        put_bits(dct[n].code, dct[n].len);
        if (n) put_bits(uint32_t(diff < 0 ? diff - 1 : diff), n);
        int last = 63;
        while (last > 0 && q[zz[last]] == 0) --last;
        int run = 0;
        for (int k = 1; k <= last; ++k) {
            const int v = q[zz[k]];
            if (v == 0) {
                ++run;
                continue;
            }
            while (run >= 16) {
                put_bits(act[0xF0].code, act[0xF0].len);
                run -= 16;
            }
            n = nbits(v);
            const int sym = (run << 4) | n;
            put_bits(act[sym].code, act[sym].len);
            put_bits(uint32_t(v < 0 ? v - 1 : v), n);
            run = 0;
        }
        if (last < 63) put_bits(act[0x00].code, act[0x00].len);
        return dcv;
    }

    uint8_t qt_[2][64];
    float fq_[2][64];
    float fqt_[2][64];   // transposed fq_
    uint8_t zzt_[64];    // transposed zigzag
    Code codes_[4][256];
    std::vector<uint8_t> out_;
    uint64_t bitbuf_ = 0;
    int bitcnt_ = 0;
};

}  // namespace roadsense
