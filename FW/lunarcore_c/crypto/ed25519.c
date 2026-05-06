#include "ed25519.h"

/* Forward declarations */
typedef struct Sha512 Sha512;

/* External dependencies - SHA-512 implementation */
extern Sha512* sha512_new(void);
extern void sha512_update(Sha512* ctx, const uint8_t* data, size_t len);
extern void sha512_finalize(Sha512* ctx, uint8_t* output);
extern void sha512_free(Sha512* ctx);
extern void sha512_hash(const uint8_t* data, size_t len, uint8_t* output);
extern bool constant_time_eq(const uint8_t* a, const uint8_t* b, size_t len);

/* Type definitions */
typedef struct {
    uint8_t data[64];
} Signature;

typedef uint8_t PublicKey[32];
typedef uint8_t PrivateKey[32];

typedef struct {
    int64_t limbs[10];
} Fe;

typedef struct {
    Fe x;
    Fe y;
    Fe z;
    Fe t;
} ExtendedPoint;

typedef struct {
    Fe y_plus_x;
    Fe y_minus_x;
    Fe xy2d;
} PrecomputedPoint;

typedef struct {
    uint8_t bytes[32];
} Scalar;

/* Constants */
static const uint8_t BASE_Y[32] = {
    0x58, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
    0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66, 0x66,
};

static const Fe D = {
    .limbs = {
        -10913610, 13857413, -15372611, 6949391, 114729,
        -8787816, -6275908, -3247719, -18696448, -12055116,
    }
};

static const Fe D2 = {
    .limbs = {
        -21827239, -5839606, -30745221, 13898782, 229458,
        15978800, -12551817, -6495438, 29715968, 9444199,
    }
};

static const Fe SQRT_M1 = {
    .limbs = {
        -32595792, -7943725, 9377950, 3500415, 12389472,
        -272473, -25146209, -2005654, 326686, 11406482,
    }
};

static const int64_t L[12] = {
    0x1cf5d3ed, 0x009318d2, 0x1de73596, 0x1df3bd45,
    0x0000014d, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00200000,
};

/* Helper functions for loading bytes */
static int64_t load_4(const uint8_t* s) {
    return (int64_t)s[0] | 
           ((int64_t)s[1] << 8) | 
           ((int64_t)s[2] << 16) | 
           ((int64_t)s[3] << 24);
}

static int64_t load_3(const uint8_t* s) {
    return (int64_t)s[0] | 
           ((int64_t)s[1] << 8) | 
           ((int64_t)s[2] << 16);
}

static int64_t load_3_i64(const uint8_t* s) {
    return (int64_t)s[0] | 
           ((int64_t)s[1] << 8) | 
           ((int64_t)s[2] << 16);
}

static int64_t load_4_i64(const uint8_t* s) {
    return (int64_t)s[0] | 
           ((int64_t)s[1] << 8) | 
           ((int64_t)s[2] << 16) | 
           ((int64_t)s[3] << 24);
}

/* Fe (Field Element) operations */
static Fe fe_zero(void) {
    Fe result;
    for (int i = 0; i < 10; i++) {
        result.limbs[i] = 0;
    }
    return result;
}

static Fe fe_one(void) {
    Fe result = fe_zero();
    result.limbs[0] = 1;
    return result;
}

static Fe fe_from_bytes(const uint8_t bytes[32]) {
    Fe h;
    
    h.limbs[0] = load_4(&bytes[0]) & 0x3ffffff;
    h.limbs[1] = (load_4(&bytes[3]) >> 2) & 0x1ffffff;
    h.limbs[2] = (load_4(&bytes[6]) >> 3) & 0x3ffffff;
    h.limbs[3] = (load_4(&bytes[9]) >> 5) & 0x1ffffff;
    h.limbs[4] = (load_4(&bytes[12]) >> 6) & 0x3ffffff;
    h.limbs[5] = load_4(&bytes[16]) & 0x1ffffff;
    h.limbs[6] = (load_4(&bytes[19]) >> 1) & 0x3ffffff;
    h.limbs[7] = (load_4(&bytes[22]) >> 3) & 0x1ffffff;
    h.limbs[8] = (load_4(&bytes[25]) >> 4) & 0x3ffffff;
    h.limbs[9] = (load_4(&bytes[28]) >> 6) & 0x1ffffff;
    
    return h;
}

static void fe_to_bytes(const Fe* self, uint8_t output[32]) {
    int64_t h[10];
    memcpy(h, self->limbs, sizeof(h));
    
    int64_t q = (19 * h[9] + (1 << 24)) >> 25;
    q = (h[0] + q) >> 26;
    q = (h[1] + q) >> 25;
    q = (h[2] + q) >> 26;
    q = (h[3] + q) >> 25;
    q = (h[4] + q) >> 26;
    q = (h[5] + q) >> 25;
    q = (h[6] + q) >> 26;
    q = (h[7] + q) >> 25;
    q = (h[8] + q) >> 26;
    q = (h[9] + q) >> 25;
    
    h[0] += 19 * q;
    
    int64_t carry0 = h[0] >> 26; h[1] += carry0; h[0] -= carry0 << 26;
    int64_t carry1 = h[1] >> 25; h[2] += carry1; h[1] -= carry1 << 25;
    int64_t carry2 = h[2] >> 26; h[3] += carry2; h[2] -= carry2 << 26;
    int64_t carry3 = h[3] >> 25; h[4] += carry3; h[3] -= carry3 << 25;
    int64_t carry4 = h[4] >> 26; h[5] += carry4; h[4] -= carry4 << 26;
    int64_t carry5 = h[5] >> 25; h[6] += carry5; h[5] -= carry5 << 25;
    int64_t carry6 = h[6] >> 26; h[7] += carry6; h[6] -= carry6 << 26;
    int64_t carry7 = h[7] >> 25; h[8] += carry7; h[7] -= carry7 << 25;
    int64_t carry8 = h[8] >> 26; h[9] += carry8; h[8] -= carry8 << 26;
    int64_t carry9 = h[9] >> 25; h[9] -= carry9 << 25;
    
    output[0] = (uint8_t)h[0];
    output[1] = (uint8_t)(h[0] >> 8);
    output[2] = (uint8_t)(h[0] >> 16);
    output[3] = (uint8_t)((h[0] >> 24) | (h[1] << 2));
    output[4] = (uint8_t)(h[1] >> 6);
    output[5] = (uint8_t)(h[1] >> 14);
    output[6] = (uint8_t)((h[1] >> 22) | (h[2] << 3));
    output[7] = (uint8_t)(h[2] >> 5);
    output[8] = (uint8_t)(h[2] >> 13);
    output[9] = (uint8_t)((h[2] >> 21) | (h[3] << 5));
    output[10] = (uint8_t)(h[3] >> 3);
    output[11] = (uint8_t)(h[3] >> 11);
    output[12] = (uint8_t)((h[3] >> 19) | (h[4] << 6));
    output[13] = (uint8_t)(h[4] >> 2);
    output[14] = (uint8_t)(h[4] >> 10);
    output[15] = (uint8_t)(h[4] >> 18);
    output[16] = (uint8_t)h[5];
    output[17] = (uint8_t)(h[5] >> 8);
    output[18] = (uint8_t)(h[5] >> 16);
    output[19] = (uint8_t)((h[5] >> 24) | (h[6] << 1));
    output[20] = (uint8_t)(h[6] >> 7);
    output[21] = (uint8_t)(h[6] >> 15);
    output[22] = (uint8_t)((h[6] >> 23) | (h[7] << 3));
    output[23] = (uint8_t)(h[7] >> 5);
    output[24] = (uint8_t)(h[7] >> 13);
    output[25] = (uint8_t)((h[7] >> 21) | (h[8] << 4));
    output[26] = (uint8_t)(h[8] >> 4);
    output[27] = (uint8_t)(h[8] >> 12);
    output[28] = (uint8_t)((h[8] >> 20) | (h[9] << 6));
    output[29] = (uint8_t)(h[9] >> 2);
    output[30] = (uint8_t)(h[9] >> 10);
    output[31] = (uint8_t)(h[9] >> 18);
}

static Fe fe_add(const Fe* self, const Fe* rhs) {
    Fe result;
    result.limbs[0] = self->limbs[0] + rhs->limbs[0];
    result.limbs[1] = self->limbs[1] + rhs->limbs[1];
    result.limbs[2] = self->limbs[2] + rhs->limbs[2];
    result.limbs[3] = self->limbs[3] + rhs->limbs[3];
    result.limbs[4] = self->limbs[4] + rhs->limbs[4];
    result.limbs[5] = self->limbs[5] + rhs->limbs[5];
    result.limbs[6] = self->limbs[6] + rhs->limbs[6];
    result.limbs[7] = self->limbs[7] + rhs->limbs[7];
    result.limbs[8] = self->limbs[8] + rhs->limbs[8];
    result.limbs[9] = self->limbs[9] + rhs->limbs[9];
    return result;
}

static Fe fe_sub(const Fe* self, const Fe* rhs) {
    Fe result;
    result.limbs[0] = self->limbs[0] - rhs->limbs[0];
    result.limbs[1] = self->limbs[1] - rhs->limbs[1];
    result.limbs[2] = self->limbs[2] - rhs->limbs[2];
    result.limbs[3] = self->limbs[3] - rhs->limbs[3];
    result.limbs[4] = self->limbs[4] - rhs->limbs[4];
    result.limbs[5] = self->limbs[5] - rhs->limbs[5];
    result.limbs[6] = self->limbs[6] - rhs->limbs[6];
    result.limbs[7] = self->limbs[7] - rhs->limbs[7];
    result.limbs[8] = self->limbs[8] - rhs->limbs[8];
    result.limbs[9] = self->limbs[9] - rhs->limbs[9];
    return result;
}

static Fe fe_neg(const Fe* self) {
    Fe result;
    result.limbs[0] = -self->limbs[0];
    result.limbs[1] = -self->limbs[1];
    result.limbs[2] = -self->limbs[2];
    result.limbs[3] = -self->limbs[3];
    result.limbs[4] = -self->limbs[4];
    result.limbs[5] = -self->limbs[5];
    result.limbs[6] = -self->limbs[6];
    result.limbs[7] = -self->limbs[7];
    result.limbs[8] = -self->limbs[8];
    result.limbs[9] = -self->limbs[9];
    return result;
}

static Fe carry_mul(__int128 h[10]) {
    Fe out;
    __int128 carry = (h[0] + (1LL << 25)) >> 26;
    out.limbs[0] = (int64_t)(h[0] - (carry << 26));
    __int128 h1 = h[1] + carry;
    carry = (h1 + (1LL << 24)) >> 25;
    out.limbs[1] = (int64_t)(h1 - (carry << 25));
    __int128 h2 = h[2] + carry;
    carry = (h2 + (1LL << 25)) >> 26;
    out.limbs[2] = (int64_t)(h2 - (carry << 26));
    __int128 h3 = h[3] + carry;
    carry = (h3 + (1LL << 24)) >> 25;
    out.limbs[3] = (int64_t)(h3 - (carry << 25));
    __int128 h4 = h[4] + carry;
    carry = (h4 + (1LL << 25)) >> 26;
    out.limbs[4] = (int64_t)(h4 - (carry << 26));
    __int128 h5 = h[5] + carry;
    carry = (h5 + (1LL << 24)) >> 25;
    out.limbs[5] = (int64_t)(h5 - (carry << 25));
    __int128 h6 = h[6] + carry;
    carry = (h6 + (1LL << 25)) >> 26;
    out.limbs[6] = (int64_t)(h6 - (carry << 26));
    __int128 h7 = h[7] + carry;
    carry = (h7 + (1LL << 24)) >> 25;
    out.limbs[7] = (int64_t)(h7 - (carry << 25));
    __int128 h8 = h[8] + carry;
    carry = (h8 + (1LL << 25)) >> 26;
    out.limbs[8] = (int64_t)(h8 - (carry << 26));
    __int128 h9 = h[9] + carry;
    carry = (h9 + (1LL << 24)) >> 25;
    out.limbs[9] = (int64_t)(h9 - (carry << 25));
    out.limbs[0] += (int64_t)(carry * 19);
    carry = ((__int128)out.limbs[0] + (1LL << 25)) >> 26;
    out.limbs[0] -= (int64_t)(carry << 26);
    out.limbs[1] += (int64_t)carry;
    return out;
}

static Fe fe_mul(const Fe* self, const Fe* rhs) {
    const int64_t* f = self->limbs;
    const int64_t* g = rhs->limbs;
    
    __int128 f0 = f[0]; __int128 f1 = f[1]; __int128 f2 = f[2];
    __int128 f3 = f[3]; __int128 f4 = f[4]; __int128 f5 = f[5];
    __int128 f6 = f[6]; __int128 f7 = f[7]; __int128 f8 = f[8];
    __int128 f9 = f[9];
    
    __int128 g0 = g[0]; __int128 g1 = g[1]; __int128 g2 = g[2];
    __int128 g3 = g[3]; __int128 g4 = g[4]; __int128 g5 = g[5];
    __int128 g6 = g[6]; __int128 g7 = g[7]; __int128 g8 = g[8];
    __int128 g9 = g[9];
    
    __int128 g1_19 = 19 * g1; __int128 g2_19 = 19 * g2; __int128 g3_19 = 19 * g3;
    __int128 g4_19 = 19 * g4; __int128 g5_19 = 19 * g5; __int128 g6_19 = 19 * g6;
    __int128 g7_19 = 19 * g7; __int128 g8_19 = 19 * g8; __int128 g9_19 = 19 * g9;
    
    __int128 f1_2 = 2 * f1; __int128 f3_2 = 2 * f3; __int128 f5_2 = 2 * f5;
    __int128 f7_2 = 2 * f7; __int128 f9_2 = 2 * f9;
    
    __int128 h[10];
    h[0] = f0*g0 + f1_2*g9_19 + f2*g8_19 + f3_2*g7_19 + f4*g6_19 + f5_2*g5_19 + f6*g4_19 + f7_2*g3_19 + f8*g2_19 + f9_2*g1_19;
    h[1] = f0*g1 + f1*g0 + f2*g9_19 + f3*g8_19 + f4*g7_19 + f5*g6_19 + f6*g5_19 + f7*g4_19 + f8*g3_19 + f9*g2_19;
    h[2] = f0*g2 + f1_2*g1 + f2*g0 + f3_2*g9_19 + f4*g8_19 + f5_2*g7_19 + f6*g6_19 + f7_2*g5_19 + f8*g4_19 + f9_2*g3_19;
    h[3] = f0*g3 + f1*g2 + f2*g1 + f3*g0 + f4*g9_19 + f5*g8_19 + f6*g7_19 + f7*g6_19 + f8*g5_19 + f9*g4_19;
    h[4] = f0*g4 + f1_2*g3 + f2*g2 + f3_2*g1 + f4*g0 + f5_2*g9_19 + f6*g8_19 + f7_2*g7_19 + f8*g6_19 + f9_2*g5_19;
    h[5] = f0*g5 + f1*g4 + f2*g3 + f3*g2 + f4*g1 + f5*g0 + f6*g9_19 + f7*g8_19 + f8*g7_19 + f9*g6_19;
    h[6] = f0*g6 + f1_2*g5 + f2*g4 + f3_2*g3 + f4*g2 + f5_2*g1 + f6*g0 + f7_2*g9_19 + f8*g8_19 + f9_2*g7_19;
    h[7] = f0*g7 + f1*g6 + f2*g5 + f3*g4 + f4*g3 + f5*g2 + f6*g1 + f7*g0 + f8*g9_19 + f9*g8_19;
    h[8] = f0*g8 + f1_2*g7 + f2*g6 + f3_2*g5 + f4*g4 + f5_2*g3 + f6*g2 + f7_2*g1 + f8*g0 + f9_2*g9_19;
    h[9] = f0*g9 + f1*g8 + f2*g7 + f3*g6 + f4*g5 + f5*g4 + f6*g3 + f7*g2 + f8*g1 + f9*g0;
    
    return carry_mul(h);
}

static Fe fe_square(const Fe* self) {
    return fe_mul(self, self);
}

static Fe fe_square2(const Fe* self) {
    Fe h = fe_square(self);
    return fe_add(&h, &h);
}

static Fe fe_invert(const Fe* self) {
    Fe t0 = fe_square(self);
    Fe t1 = fe_square(&t0);
    t1 = fe_square(&t1);
    t1 = fe_mul(self, &t1);
    t0 = fe_mul(&t0, &t1);
    Fe t2 = fe_square(&t0);
    t1 = fe_mul(&t1, &t2);
    t2 = fe_square(&t1);
    for (int i = 1; i < 5; i++) { t2 = fe_square(&t2); }
    t1 = fe_mul(&t2, &t1);
    t2 = fe_square(&t1);
    for (int i = 1; i < 10; i++) { t2 = fe_square(&t2); }
    t2 = fe_mul(&t2, &t1);
    Fe t3 = fe_square(&t2);
    for (int i = 1; i < 20; i++) { t3 = fe_square(&t3); }
    t2 = fe_mul(&t3, &t2);
    t2 = fe_square(&t2);
    for (int i = 1; i < 10; i++) { t2 = fe_square(&t2); }
    t1 = fe_mul(&t2, &t1);
    t2 = fe_square(&t1);
    for (int i = 1; i < 50; i++) { t2 = fe_square(&t2); }
    t2 = fe_mul(&t2, &t1);
    t3 = fe_square(&t2);
    for (int i = 1; i < 100; i++) { t3 = fe_square(&t3); }
    t2 = fe_mul(&t3, &t2);
    t2 = fe_square(&t2);
    for (int i = 1; i < 50; i++) { t2 = fe_square(&t2); }
    t1 = fe_mul(&t2, &t1);
    t1 = fe_square(&t1);
    for (int i = 1; i < 5; i++) { t1 = fe_square(&t1); }
    return fe_mul(&t0, &t1);
}

static Fe fe_pow22523(const Fe* self) {
    Fe t0 = fe_square(self);
    Fe t1 = fe_square(&t0);
    t1 = fe_square(&t1);
    t1 = fe_mul(self, &t1);
    t0 = fe_mul(&t0, &t1);
    t0 = fe_square(&t0);
    t0 = fe_mul(&t1, &t0);
    t1 = fe_square(&t0);
    for (int i = 1; i < 5; i++) { t1 = fe_square(&t1); }
    t0 = fe_mul(&t1, &t0);
    t1 = fe_square(&t0);
    for (int i = 1; i < 10; i++) { t1 = fe_square(&t1); }
    t1 = fe_mul(&t1, &t0);
    Fe t2 = fe_square(&t1);
    for (int i = 1; i < 20; i++) { t2 = fe_square(&t2); }
    t1 = fe_mul(&t2, &t1);
    t1 = fe_square(&t1);
    for (int i = 1; i < 10; i++) { t1 = fe_square(&t1); }
    t0 = fe_mul(&t1, &t0);
    t1 = fe_square(&t0);
    for (int i = 1; i < 50; i++) { t1 = fe_square(&t1); }
    t1 = fe_mul(&t1, &t0);
    t2 = fe_square(&t1);
    for (int i = 1; i < 100; i++) { t2 = fe_square(&t2); }
    t1 = fe_mul(&t2, &t1);
    t1 = fe_square(&t1);
    for (int i = 1; i < 50; i++) { t1 = fe_square(&t1); }
    t0 = fe_mul(&t1, &t0);
    t0 = fe_square(&t0);
    t0 = fe_square(&t0);
    return fe_mul(self, &t0);
}

static bool fe_is_negative(const Fe* self) {
    uint8_t bytes[32];
    fe_to_bytes(self, bytes);
    return (bytes[0] & 1) == 1;
}

static bool fe_is_zero(const Fe* self) {
    uint8_t bytes[32];
    fe_to_bytes(self, bytes);
    uint8_t zero[32] = {0};
    return memcmp(bytes, zero, 32) == 0;
}

static Fe fe_abs(const Fe* self) {
    if (fe_is_negative(self)) {
        return fe_neg(self);
    } else {
        return *self;
    }
}

/* Constant-time operations */
static void fe_ct_swap(Fe* a, Fe* b, int64_t mask) {
    for (int i = 0; i < 10; i++) {
        int64_t t = mask & (a->limbs[i] ^ b->limbs[i]);
        a->limbs[i] ^= t;
        b->limbs[i] ^= t;
    }
}

static Fe fe_ct_select(const Fe* a, const Fe* b, int64_t mask) {
    Fe result = fe_zero();
    for (int i = 0; i < 10; i++) {
        result.limbs[i] = a->limbs[i] ^ (mask & (a->limbs[i] ^ b->limbs[i]));
    }
    return result;
}

/* ExtendedPoint operations */
static ExtendedPoint extended_point_identity(void) {
    ExtendedPoint result;
    result.x = fe_zero();
    result.y = fe_one();
    result.z = fe_one();
    result.t = fe_zero();
    return result;
}

static bool extended_point_from_bytes(const uint8_t bytes[32], ExtendedPoint* result) {
    Fe y = fe_from_bytes(bytes);
    Fe z = fe_one();
    Fe y2 = fe_square(&y);
    Fe u = fe_sub(&y2, &fe_one());
    Fe v = fe_add(&fe_mul(&y2, &D), &fe_one());
    Fe v3 = fe_mul(&fe_square(&v), &v);
    Fe v7 = fe_mul(&fe_square(&v3), &v);
    Fe uv7 = fe_mul(&u, &v7);
    Fe x = fe_mul(&fe_mul(&fe_pow22523(&uv7), &u), &v3);
    
    Fe vx2 = fe_mul(&fe_square(&x), &v);
    Fe check = fe_sub(&vx2, &u);
    if (!fe_is_zero(&check)) {
        Fe check2 = fe_add(&vx2, &u);
        if (!fe_is_zero(&check2)) {
            return false;
        }
        x = fe_mul(&x, &SQRT_M1);
    }
    
    if (fe_is_negative(&x) != ((bytes[31] >> 7) == 1)) {
        x = fe_neg(&x);
    }
    
    Fe t = fe_mul(&x, &y);
    result->x = x;
    result->y = y;
    result->z = z;
    result->t = t;
    return true;
}

static void extended_point_to_bytes(const ExtendedPoint* self, uint8_t output[32]) {
    Fe z_inv = fe_invert(&self->z);
    Fe x = fe_mul(&self->x, &z_inv);
    Fe y = fe_mul(&self->y, &z_inv);
    fe_to_bytes(&y, output);
    output[31] ^= (fe_is_negative(&x) ? 1 : 0) << 7;
}

static ExtendedPoint extended_point_double(const ExtendedPoint* self) {
    Fe a = fe_square(&self->x);
    Fe b = fe_square(&self->y);
    Fe c = fe_square2(&self->z);
    Fe d = fe_neg(&a);
    Fe e = fe_sub(&fe_sub(&fe_square(&fe_add(&self->x, &self->y)), &a), &b);
    Fe g = fe_add(&d, &b);
    Fe f = fe_sub(&g, &c);
    Fe h = fe_sub(&d, &b);
    Fe x3 = fe_mul(&e, &f);
    Fe y3 = fe_mul(&g, &h);
    Fe t3 = fe_mul(&e, &h);
    Fe z3 = fe_mul(&f, &g);
    
    ExtendedPoint result;
    result.x = x3;
    result.y = y3;
    result.z = z3;
    result.t = t3;
    return result;
}

static ExtendedPoint extended_point_add(const ExtendedPoint* self, const ExtendedPoint* other) {
    Fe a = fe_mul(&fe_sub(&self->y, &self->x), &fe_sub(&other->y, &other->x));
    Fe b = fe_mul(&fe_add(&self->y, &self->x), &fe_add(&other->y, &other->x));
    Fe c = fe_mul(&fe_mul(&self->t, &D2), &other->t);
    Fe d = fe_add(&fe_mul(&self->z, &other->z), &fe_mul(&self->z, &other->z));
    Fe e = fe_sub(&b, &a);
    Fe f = fe_sub(&d, &c);
    Fe g = fe_add(&d, &c);
    Fe h = fe_add(&b, &a);
    Fe x3 = fe_mul(&e, &f);
    Fe y3 = fe_mul(&g, &h);
    Fe t3 = fe_mul(&e, &h);
    Fe z3 = fe_mul(&f, &g);
    
    ExtendedPoint result;
    result.x = x3;
    result.y = y3;
    result.z = z3;
    result.t = t3;
    return result;
}

static ExtendedPoint extended_point_neg(const ExtendedPoint* self) {
    ExtendedPoint result;
    result.x = fe_neg(&self->x);
    result.y = self->y;
    result.z = self->z;
    result.t = fe_neg(&self->t);
    return result;
}

static void ct_swap(ExtendedPoint* a, ExtendedPoint* b, int64_t choice) {
    int64_t mask = -(choice);
    fe_ct_swap(&a->x, &b->x, mask);
    fe_ct_swap(&a->y, &b->y, mask);
    fe_ct_swap(&a->z, &b->z, mask);
    fe_ct_swap(&a->t, &b->t, mask);
}

static ExtendedPoint extended_point_ct_select(const ExtendedPoint* a, const ExtendedPoint* b, int64_t choice) {
    int64_t mask = -(choice);
    ExtendedPoint result;
    result.x = fe_ct_select(&a->x, &b->x, mask);
    result.y = fe_ct_select(&a->y, &b->y, mask);
    result.z = fe_ct_select(&a->z, &b->z, mask);
    result.t = fe_ct_select(&a->t, &b->t, mask);
    return result;
}

static ExtendedPoint extended_point_scalar_mul(const ExtendedPoint* self, const uint8_t scalar[32]) {
    ExtendedPoint r0 = extended_point_identity();
    ExtendedPoint r1 = *self;
    
    for (int i = 255; i >= 0; i--) {
        int byte_idx = i / 8;
        int bit_idx = i % 8;
        int64_t bit = ((scalar[byte_idx] >> bit_idx) & 1);
        
        ct_swap(&r0, &r1, bit);
        
        r1 = extended_point_add(&r0, &r1);
        r0 = extended_point_double(&r0);
        
        ct_swap(&r0, &r1, bit);
    }
    
    return r0;
}

static ExtendedPoint basepoint(void) {
    ExtendedPoint result;
    extended_point_from_bytes(BASE_Y, &result);
    return result;
}

/* Scalar operations */
static void sc_reduce_limbs(int64_t a[24]) {
    for (int i = 23; i >= 12; i--) {
        int64_t q = a[i];
        if (q == 0) continue;
        
        int shift = i - 11;
        a[shift + 0] -= q * 0x1cf5d3ed;
        a[shift + 1] -= q * 0x009318d2;
        a[shift + 2] -= q * 0x1de73596;
        a[shift + 3] -= q * 0x1df3bd45;
        a[shift + 4] -= q * 0x0000014d;
        
        a[shift + 11] -= q * 0x00200000;
        a[i] = 0;
    }
    
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 11; i++) {
            int64_t carry = a[i] >> 21;
            a[i] &= 0x1fffff;
            a[i + 1] += carry;
        }
        
        for (int i = 0; i < 11; i++) {
            if (a[i] < 0) {
                a[i] += 0x200000;
                a[i + 1] -= 1;
            }
        }
    }
    
    int64_t borrow = 0;
    int64_t tmp[12];
    
    tmp[0] = a[0] - 0x1cf5d3ed;
    tmp[1] = a[1] - 0x009318d2;
    tmp[2] = a[2] - 0x1de73596;
    tmp[3] = a[3] - 0x1df3bd45;
    tmp[4] = a[4] - 0x0000014d;
    tmp[5] = a[5];
    tmp[6] = a[6];
    tmp[7] = a[7];
    tmp[8] = a[8];
    tmp[9] = a[9];
    tmp[10] = a[10];
    tmp[11] = a[11] - 0x00200000;
    
    for (int i = 0; i < 11; i++) {
        tmp[i] += borrow;
        borrow = tmp[i] >> 63;
        if (tmp[i] < 0) {
            tmp[i] += 0x200000;
            borrow = -1;
        } else {
            borrow = 0;
        }
    }
    tmp[11] += borrow;
    
    int64_t mask = ~(tmp[11] >> 63);
    for (int i = 0; i < 12; i++) {
        a[i] = (a[i] & ~mask) | (tmp[i] & mask);
    }
}

static void sc_reduce(const uint8_t s[64], uint8_t out[32]) {
    int64_t a[24] = {0};
    
    a[0] = 0x1fffff & load_3_i64(&s[0]);
    a[1] = 0x1fffff & (load_4_i64(&s[2]) >> 5);
    a[2] = 0x1fffff & (load_3_i64(&s[5]) >> 2);
    a[3] = 0x1fffff & (load_4_i64(&s[7]) >> 7);
    a[4] = 0x1fffff & (load_4_i64(&s[10]) >> 4);
    a[5] = 0x1fffff & (load_3_i64(&s[13]) >> 1);
    a[6] = 0x1fffff & (load_4_i64(&s[15]) >> 6);
    a[7] = 0x1fffff & (load_3_i64(&s[18]) >> 3);
    a[8] = 0x1fffff & load_3_i64(&s[21]);
    a[9] = 0x1fffff & (load_4_i64(&s[23]) >> 5);
    a[10] = 0x1fffff & (load_3_i64(&s[26]) >> 2);
    a[11] = 0x1fffff & (load_4_i64(&s[28]) >> 7);
    a[12] = 0x1fffff & (load_4_i64(&s[31]) >> 4);
    a[13] = 0x1fffff & (load_3_i64(&s[34]) >> 1);
    a[14] = 0x1fffff & (load_4_i64(&s[36]) >> 6);
    a[15] = 0x1fffff & (load_3_i64(&s[39]) >> 3);
    a[16] = 0x1fffff & load_3_i64(&s[42]);
    a[17] = 0x1fffff & (load_4_i64(&s[44]) >> 5);
    a[18] = 0x1fffff & (load_3_i64(&s[47]) >> 2);
    a[19] = 0x1fffff & (load_4_i64(&s[49]) >> 7);
    a[20] = 0x1fffff & (load_4_i64(&s[52]) >> 4);
    a[21] = 0x1fffff & (load_3_i64(&s[55]) >> 1);
    a[22] = 0x1fffff & (load_4_i64(&s[57]) >> 6);
    a[23] = load_4_i64(&s[60]) >> 3;
    
    sc_reduce_limbs(a);
    
    out[0] = (uint8_t)a[0];
    out[1] = (uint8_t)(a[0] >> 8);
    out[2] = (uint8_t)((a[0] >> 16) | (a[1] << 5));
    out[3] = (uint8_t)(a[1] >> 3);
    out[4] = (uint8_t)(a[1] >> 11);
    out[5] = (uint8_t)((a[1] >> 19) | (a[2] << 2));
    out[6] = (uint8_t)(a[2] >> 6);
    out[7] = (uint8_t)((a[2] >> 14) | (a[3] << 7));
    out[8] = (uint8_t)(a[3] >> 1);
    out[9] = (uint8_t)(a[3] >> 9);
    out[10] = (uint8_t)((a[3] >> 17) | (a[4] << 4));
    out[11] = (uint8_t)(a[4] >> 4);
    out[12] = (uint8_t)(a[4] >> 12);
    out[13] = (uint8_t)((a[4] >> 20) | (a[5] << 1));
    out[14] = (uint8_t)(a[5] >> 7);
    out[15] = (uint8_t)((a[5] >> 15) | (a[6] << 6));
    out[16] = (uint8_t)(a[6] >> 2);
    out[17] = (uint8_t)(a[6] >> 10);
    out[18] = (uint8_t)((a[6] >> 18) | (a[7] << 3));
    out[19] = (uint8_t)(a[7] >> 5);
    out[20] = (uint8_t)(a[7] >> 13);
    out[21] = (uint8_t)a[8];
    out[22] = (uint8_t)(a[8] >> 8);
    out[23] = (uint8_t)((a[8] >> 16) | (a[9] << 5));
    out[24] = (uint8_t)(a[9] >> 3);
    out[25] = (uint8_t)(a[9] >> 11);
    out[26] = (uint8_t)((a[9] >> 19) | (a[10] << 2));
    out[27] = (uint8_t)(a[10] >> 6);
    out[28] = (uint8_t)((a[10] >> 14) | (a[11] << 7));
    out[29] = (uint8_t)(a[11] >> 1);
    out[30] = (uint8_t)(a[11] >> 9);
    out[31] = (uint8_t)(a[11] >> 17);
}

static void sc_load(const uint8_t s[32], int64_t out[12]) {
    out[0] = 0x1fffff & load_3_i64(&s[0]);
    out[1] = 0x1fffff & (load_4_i64(&s[2]) >> 5);
    out[2] = 0x1fffff & (load_3_i64(&s[5]) >> 2);
    out[3] = 0x1fffff & (load_4_i64(&s[7]) >> 7);
    out[4] = 0x1fffff & (load_4_i64(&s[10]) >> 4);
    out[5] = 0x1fffff & (load_3_i64(&s[13]) >> 1);
    out[6] = 0x1fffff & (load_4_i64(&s[15]) >> 6);
    out[7] = 0x1fffff & (load_3_i64(&s[18]) >> 3);
    out[8] = 0x1fffff & load_3_i64(&s[21]);
    out[9] = 0x1fffff & (load_4_i64(&s[23]) >> 5);
    out[10] = 0x1fffff & (load_3_i64(&s[26]) >> 2);
    out[11] = (load_4_i64(&s[28]) >> 7);
}

static void sc_muladd(const uint8_t a[32], const uint8_t b[32], const uint8_t c[32], uint8_t out[32]) {
    int64_t a_limbs[12];
    int64_t b_limbs[12];
    int64_t c_limbs[12];
    
    sc_load(a, a_limbs);
    sc_load(b, b_limbs);
    sc_load(c, c_limbs);
    
    int64_t product[24] = {0};
    for (int i = 0; i < 12; i++) {
        for (int j = 0; j < 12; j++) {
            product[i + j] += a_limbs[i] * b_limbs[j];
        }
    }
    
    for (int i = 0; i < 12; i++) {
        product[i] += c_limbs[i];
    }
    
    for (int i = 0; i < 23; i++) {
        int64_t carry = product[i] >> 21;
        product[i] &= 0x1fffff;
        product[i + 1] += carry;
    }
    
    sc_reduce_limbs(product);
    
    out[0] = (uint8_t)product[0];
    out[1] = (uint8_t)(product[0] >> 8);
    out[2] = (uint8_t)((product[0] >> 16) | (product[1] << 5));
    out[3] = (uint8_t)(product[1] >> 3);
    out[4] = (uint8_t)(product[1] >> 11);
    out[5] = (uint8_t)((product[1] >> 19) | (product[2] << 2));
    out[6] = (uint8_t)(product[2] >> 6);
    out[7] = (uint8_t)((product[2] >> 14) | (product[3] << 7));
    out[8] = (uint8_t)(product[3] >> 1);
    out[9] = (uint8_t)(product[3] >> 9);
    out[10] = (uint8_t)((product[3] >> 17) | (product[4] << 4));
    out[11] = (uint8_t)(product[4] >> 4);
    out[12] = (uint8_t)(product[4] >> 12);
    out[13] = (uint8_t)((product[4] >> 20) | (product[5] << 1));
    out[14] = (uint8_t)(product[5] >> 7);
    out[15] = (uint8_t)((product[5] >> 15) | (product[6] << 6));
    out[16] = (uint8_t)(product[6] >> 2);
    out[17] = (uint8_t)(product[6] >> 10);
    out[18] = (uint8_t)((product[6] >> 18) | (product[7] << 3));
    out[19] = (uint8_t)(product[7] >> 5);
    out[20] = (uint8_t)(product[7] >> 13);
    out[21] = (uint8_t)product[8];
    out[22] = (uint8_t)(product[8] >> 8);
    out[23] = (uint8_t)((product[8] >> 16) | (product[9] << 5));
    out[24] = (uint8_t)(product[9] >> 3);
    out[25] = (uint8_t)(product[9] >> 11);
    out[26] = (uint8_t)((product[9] >> 19) | (product[10] << 2));
    out[27] = (uint8_t)(product[10] >> 6);
    out[28] = (uint8_t)((product[10] >> 14) | (product[11] << 7));
    out[29] = (uint8_t)(product[11] >> 1);
    out[30] = (uint8_t)(product[11] >> 9);
    out[31] = (uint8_t)(product[11] >> 17);
}

static bool sc_is_valid(const uint8_t s[32]) {
    /* Check if scalar is less than L (the group order) */
    for (int i = 31; i >= 0; i--) {
        int byte_idx = i;
        int limb_idx = -1;
        int shift = 0;
        
        /* Map byte index to limb boundaries for comparison */
        if (i >= 28) {
            limb_idx = 11;
            uint64_t val = ((uint64_t)s[31] << 24) | ((uint64_t)s[30] << 16) | 
                          ((uint64_t)s[29] << 8) | s[28];
            val >>= 7;
            if (i == 31) {
                if ((val >> 17) > (L[11] >> 17)) return false;
                if ((val >> 17) < (L[11] >> 17)) return true;
            }
            continue;
        }
    }
    
    /* Full comparison with L */
    uint64_t carry = 0;
    for (int i = 0; i < 12; i++) {
        int64_t limb;
        if (i < 11) {
            /* Load limb from bytes */
            switch(i) {
                case 0: limb = 0x1fffff & load_3_i64(&s[0]); break;
                case 1: limb = 0x1fffff & (load_4_i64(&s[2]) >> 5); break;
                case 2: limb = 0x1fffff & (load_3_i64(&s[5]) >> 2); break;
                case 3: limb = 0x1fffff & (load_4_i64(&s[7]) >> 7); break;
                case 4: limb = 0x1fffff & (load_4_i64(&s[10]) >> 4); break;
                case 5: limb = 0x1fffff & (load_3_i64(&s[13]) >> 1); break;
                case 6: limb = 0x1fffff & (load_4_i64(&s[15]) >> 6); break;
                case 7: limb = 0x1fffff & (load_3_i64(&s[18]) >> 3); break;
                case 8: limb = 0x1fffff & load_3_i64(&s[21]); break;
                case 9: limb = 0x1fffff & (load_4_i64(&s[23]) >> 5); break;
                case 10: limb = 0x1fffff & (load_3_i64(&s[26]) >> 2); break;
                default: limb = 0; break;
            }
        } else {
            limb = (load_4_i64(&s[28]) >> 7);
        }
        
        uint64_t diff = (uint64_t)limb - (uint64_t)L[i] - carry;
        carry = (diff >> 63) & 1;
        
        if (i == 11 && carry == 0) return true;
    }
    
    return carry != 0;
}

/* SHA-512 wrapper (assumes external implementation) */
static void sha512(const uint8_t* data, size_t len, uint8_t output[64]) {
    sha512_hash(data, len, output);
}

/* Ed25519 public API */
void ed25519_public_key(const PrivateKey private_key, PublicKey public_key) {
    uint8_t h[64];
    sha512(private_key, 32, h);
    
    uint8_t s[32];
    memcpy(s, h, 32);
    s[0] &= 248;
    s[31] &= 127;
    s[31] |= 64;
    
    ExtendedPoint b = basepoint();
    ExtendedPoint a = extended_point_scalar_mul(&b, s);
    extended_point_to_bytes(&a, public_key);
}

void ed25519_sign(const PrivateKey private_key, const uint8_t* message, size_t message_len, Signature* signature) {
    uint8_t h[64];
    sha512(private_key, 32, h);
    
    uint8_t s[32];
    memcpy(s, h, 32);
    s[0] &= 248;
    s[31] &= 127;
    s[31] |= 64;
    
    ExtendedPoint b = basepoint();
    ExtendedPoint a_point = extended_point_scalar_mul(&b, s);
    uint8_t a[32];
    extended_point_to_bytes(&a_point, a);
    
    /* Compute r = H(h[32:64] || message) */
    Sha512* hasher_r = sha512_new();
    sha512_update(hasher_r, &h[32], 32);
    sha512_update(hasher_r, message, message_len);
    uint8_t r_hash[64];
    sha512_finalize(hasher_r, r_hash);
    sha512_free(hasher_r);
    
    uint8_t r[32];
    sc_reduce(r_hash, r);
    
    ExtendedPoint r_point = extended_point_scalar_mul(&b, r);
    uint8_t r_bytes[32];
    extended_point_to_bytes(&r_point, r_bytes);
    
    /* Compute k = H(R || A || message) */
    Sha512* hasher_k = sha512_new();
    sha512_update(hasher_k, r_bytes, 32);
    sha512_update(hasher_k, a, 32);
    sha512_update(hasher_k, message, message_len);
    uint8_t k_hash[64];
    sha512_finalize(hasher_k, k_hash);
    sha512_free(hasher_k);
    
    uint8_t k[32];
    sc_reduce(k_hash, k);
    
    /* Compute S = (r + k * s) mod L */
    uint8_t s_scalar[32];
    sc_muladd(k, s, r, s_scalar);
    
    memcpy(signature->data, r_bytes, 32);
    memcpy(&signature->data[32], s_scalar, 32);
}

bool ed25519_verify(const PublicKey public_key, const uint8_t* message, size_t message_len, const Signature* signature) {
    ExtendedPoint a_point;
    if (!extended_point_from_bytes(public_key, &a_point)) {
        return false;
    }
    
    const uint8_t* s = &signature->data[32];
    if (!sc_is_valid(s)) {
        return false;
    }
    
    /* Compute k = H(R || A || message) */
    Sha512* hasher = sha512_new();
    sha512_update(hasher, signature->data, 32);
    sha512_update(hasher, public_key, 32);
    sha512_update(hasher, message, message_len);
    uint8_t k_hash[64];
    sha512_finalize(hasher, k_hash);
    sha512_free(hasher);
    
    uint8_t k[32];
    sc_reduce(k_hash, k);
    
    ExtendedPoint b = basepoint();
    uint8_t s_bytes[32];
    memcpy(s_bytes, s, 32);
    ExtendedPoint sb = extended_point_scalar_mul(&b, s_bytes);
    
    ExtendedPoint r_point;
    if (!extended_point_from_bytes(signature->data, &r_point)) {
        return false;
    }
    
    ExtendedPoint ka = extended_point_scalar_mul(&a_point, k);
    ExtendedPoint rhs = extended_point_add(&r_point, &ka);
    
    uint8_t lhs_bytes[32];
    uint8_t rhs_bytes[32];
    extended_point_to_bytes(&sb, lhs_bytes);
    extended_point_to_bytes(&rhs, rhs_bytes);
    
    return constant_time_eq(lhs_bytes, rhs_bytes, 32);
}

////////////////////////////////////////////////////////////////////////////

/* Global constant for Ed25519 scalar field modulus limbs (21-bit base) */
const int64_t L_LIMBS[12] = 
{
  0x1cf5d3ed, 0x009318d2, 0x1de73596, 0x1df3bd45,
  0x0000014d, 0x00000000, 0x00000000, 0x00000000,
  0x00000000, 0x00000000, 0x00000000, 0x00200000,
};

/* Forward declarations for helper functions */
static int64_t load_3_i64(const uint8_t *s);
static int64_t load_4_i64(const uint8_t *s);
static void sc_reduce_limbs(int64_t a[24]);
static void sc_load(int64_t a[12], const uint8_t s[32]);

/**
 * Reduces a 64-byte scalar modulo the group order L.
 * Returns the 32-byte reduced result.
 */
void sc_reduce(const uint8_t s[64], uint8_t out[32]) 
{
  int64_t a[24];
  a[0] = 0x1fffff & load_3_i64(&s[0]);
  a[1] = 0x1fffff & (load_4_i64(&s[2]) >> 5);
  a[2] = 0x1fffff & (load_3_i64(&s[5]) >> 2);
  a[3] = 0x1fffff & (load_4_i64(&s[7]) >> 7);
  a[4] = 0x1fffff & (load_4_i64(&s[10]) >> 4);
  a[5] = 0x1fffff & (load_3_i64(&s[13]) >> 1);
  a[6] = 0x1fffff & (load_4_i64(&s[15]) >> 6);
  a[7] = 0x1fffff & (load_3_i64(&s[18]) >> 3);
  a[8] = 0x1fffff & load_3_i64(&s[21]);
  a[9] = 0x1fffff & (load_4_i64(&s[23]) >> 5);
  a[10] = 0x1fffff & (load_3_i64(&s[26]) >> 2);
  a[11] = 0x1fffff & (load_4_i64(&s[28]) >> 7);
  a[12] = 0x1fffff & (load_4_i64(&s[31]) >> 4);
  a[13] = 0x1fffff & (load_3_i64(&s[34]) >> 1);
  a[14] = 0x1fffff & (load_4_i64(&s[36]) >> 6);
  a[15] = 0x1fffff & (load_3_i64(&s[39]) >> 3);
  a[16] = 0x1fffff & load_3_i64(&s[42]);
  a[17] = 0x1fffff & (load_4_i64(&s[44]) >> 5);
  a[18] = 0x1fffff & (load_3_i64(&s[47]) >> 2);
  a[19] = 0x1fffff & (load_4_i64(&s[49]) >> 7);
  a[20] = 0x1fffff & (load_4_i64(&s[52]) >> 4);
  a[21] = 0x1fffff & (load_3_i64(&s[55]) >> 1);
  a[22] = 0x1fffff & (load_4_i64(&s[57]) >> 6);
  a[23] = load_4_i64(&s[60]) >> 3;
  sc_reduce_limbs(a);
  out[0] = (uint8_t)a[0];
  out[1] = (uint8_t)(a[0] >> 8);
  out[2] = (uint8_t)((a[0] >> 16) | (a[1] << 5));
  out[3] = (uint8_t)(a[1] >> 3);
  out[4] = (uint8_t)(a[1] >> 11);
  out[5] = (uint8_t)((a[1] >> 19) | (a[2] << 2));
  out[6] = (uint8_t)(a[2] >> 6);
  out[7] = (uint8_t)((a[2] >> 14) | (a[3] << 7));
  out[8] = (uint8_t)(a[3] >> 1);
  out[9] = (uint8_t)(a[3] >> 9);
  out[10] = (uint8_t)((a[3] >> 17) | (a[4] << 4));
  out[11] = (uint8_t)(a[4] >> 4);
  out[12] = (uint8_t)(a[4] >> 12);
  out[13] = (uint8_t)((a[4] >> 20) | (a[5] << 1));
  out[14] = (uint8_t)(a[5] >> 7);
  out[15] = (uint8_t)((a[5] >> 15) | (a[6] << 6));
  out[16] = (uint8_t)(a[6] >> 2);
  out[17] = (uint8_t)(a[6] >> 10);
  out[18] = (uint8_t)((a[6] >> 18) | (a[7] << 3));
  out[19] = (uint8_t)(a[7] >> 5);
  out[20] = (uint8_t)(a[7] >> 13);
  out[21] = (uint8_t)a[8];
  out[22] = (uint8_t)(a[8] >> 8);
  out[23] = (uint8_t)((a[8] >> 16) | (a[9] << 5));
  out[24] = (uint8_t)(a[9] >> 3);
  out[25] = (uint8_t)(a[9] >> 11);
  out[26] = (uint8_t)((a[9] >> 19) | (a[10] << 2));
  out[27] = (uint8_t)(a[10] >> 6);
  out[28] = (uint8_t)((a[10] >> 14) | (a[11] << 7));
  out[29] = (uint8_t)(a[11] >> 1);
  out[30] = (uint8_t)(a[11] >> 9);
  out[31] = (uint8_t)(a[11] >> 17);
}

/**
 * Core limb reduction logic for the Ed25519 scalar field modulus L.
 */
static void sc_reduce_limbs(int64_t a[24]) 
{
  for (int i = 23; i >= 12; i--) 
  {
    int64_t q = a[i];
    if (q == 0) continue;
    int shift = i - 11;
    a[shift + 0] -= q * 0x1cf5d3ed;
    a[shift + 1] -= q * 0x009318d2;
    a[shift + 2] -= q * 0x1de73596;
    a[shift + 3] -= q * 0x1df3bd45;
    a[shift + 4] -= q * 0x0000014d;
    a[shift + 11] -= q * 0x00200000;
    a[i] = 0;
  }
  for (int iter = 0; iter < 2; iter++) 
  {
    for (int i = 0; i < 11; i++) 
    {
      int64_t carry = a[i] >> 21;
      a[i] &= 0x1fffff;
      a[i + 1] += carry;
    }
    for (int i = 0; i < 11; i++) 
    {
      if (a[i] < 0) 
      {
        a[i] += 0x200000;
        a[i + 1] -= 1;
      }
    }
  }
  int64_t borrow = 0;
  int64_t tmp[12];
  tmp[0] = a[0] - 0x1cf5d3ed;
  tmp[1] = a[1] - 0x009318d2;
  tmp[2] = a[2] - 0x1de73596;
  tmp[3] = a[3] - 0x1df3bd45;
  tmp[4] = a[4] - 0x0000014d;
  tmp[5] = a[5];
  tmp[6] = a[6];
  tmp[7] = a[7];
  tmp[8] = a[8];
  tmp[9] = a[9];
  tmp[10] = a[10];
  tmp[11] = a[11] - 0x00200000;
  for (int i = 0; i < 11; i++) 
  {
    tmp[i] += borrow;
    borrow = tmp[i] >> 63;
    if (tmp[i] < 0) 
    {
      tmp[i] += 0x200000;
      borrow = -1;
    } 
    else borrow = 0;
  }
  tmp[11] += borrow;
  int64_t mask = ~(tmp[11] >> 63);
  for (int i = 0; i < 12; i++) a[i] = (a[i] & ~mask) | (tmp[i] & mask);
}

static int64_t load_3_i64(const uint8_t *s) 
{
  return ((int64_t)s[0]) | (((int64_t)s[1]) << 8) | (((int64_t)s[2]) << 16);
}

static int64_t load_4_i64(const uint8_t *s) 
{
  return ((int64_t)s[0]) | (((int64_t)s[1]) << 8) | (((int64_t)s[2]) << 16) | (((int64_t)s[3]) << 24);
}

/**
 * Performs (a * b) + c modulo L.
 */
void sc_muladd(const uint8_t a[32], const uint8_t b[32], const uint8_t c[32], uint8_t out[32]) 
{
  int64_t a_limbs[12];
  int64_t b_limbs[12];
  int64_t c_limbs[12];
  sc_load(a_limbs, a);
  sc_load(b_limbs, b);
  sc_load(c_limbs, c);
  int64_t product[24];
  memset(product, 0, sizeof(product));
  for (int i = 0; i < 12; i++) 
  {
    for (int j = 0; j < 12; j++) product[i + j] += a_limbs[i] * b_limbs[j];
  }
  for (int i = 0; i < 12; i++) product[i] += c_limbs[i];
  for (int i = 0; i < 23; i++) 
  {
    int64_t carry = product[i] >> 21;
    product[i] &= 0x1fffff;
    product[i + 1] += carry;
  }
  sc_reduce_limbs(product);
  out[0] = (uint8_t)product[0];
  out[1] = (uint8_t)(product[0] >> 8);
  out[2] = (uint8_t)((product[0] >> 16) | (product[1] << 5));
  out[3] = (uint8_t)(product[1] >> 3);
  out[4] = (uint8_t)(product[1] >> 11);
  out[5] = (uint8_t)((product[1] >> 19) | (product[2] << 2));
  out[6] = (uint8_t)(product[2] >> 6);
  out[7] = (uint8_t)((product[2] >> 14) | (product[3] << 7));
  out[8] = (uint8_t)(product[3] >> 1);
  out[9] = (uint8_t)(product[3] >> 9);
  out[10] = (uint8_t)((product[3] >> 17) | (product[4] << 4));
  out[11] = (uint8_t)(product[4] >> 4);
  out[12] = (uint8_t)(product[4] >> 12);
  out[13] = (uint8_t)((product[4] >> 20) | (product[5] << 1));
  out[14] = (uint8_t)(product[5] >> 7);
  out[15] = (uint8_t)((product[5] >> 15) | (product[6] << 6));
  out[16] = (uint8_t)(product[6] >> 2);
  out[17] = (uint8_t)(product[6] >> 10);
  out[18] = (uint8_t)((product[6] >> 18) | (product[7] << 3));
  out[19] = (uint8_t)(product[7] >> 5);
  out[20] = (uint8_t)(product[7] >> 13);
  out[21] = (uint8_t)product[8];
  out[22] = (uint8_t)(product[8] >> 8);
  out[23] = (uint8_t)((product[8] >> 16) | (product[9] << 5));
  out[24] = (uint8_t)(product[9] >> 3);
  out[25] = (uint8_t)(product[9] >> 11);
  out[26] = (uint8_t)((product[9] >> 19) | (product[10] << 2));
  out[27] = (uint8_t)(product[10] >> 6);
  out[28] = (uint8_t)((product[10] >> 14) | (product[11] << 7));
  out[29] = (uint8_t)(product[11] >> 1);
  out[30] = (uint8_t)(product[11] >> 9);
  out[31] = (uint8_t)(product[11] >> 17);
}

static void sc_load(int64_t a[12], const uint8_t s[32]) 
{
  a[0] = 0x1fffff & load_3_i64(&s[0]);
  a[1] = 0x1fffff & (load_4_i64(&s[2]) >> 5);
  a[2] = 0x1fffff & (load_3_i64(&s[5]) >> 2);
  a[3] = 0x1fffff & (load_4_i64(&s[7]) >> 7);
  a[4] = 0x1fffff & (load_4_i64(&s[10]) >> 4);
  a[5] = 0x1fffff & (load_3_i64(&s[13]) >> 1);
  a[6] = 0x1fffff & (load_4_i64(&s[15]) >> 6);
  a[7] = 0x1fffff & (load_3_i64(&s[18]) >> 3);
  a[8] = 0x1fffff & load_3_i64(&s[21]);
  a[9] = 0x1fffff & (load_4_i64(&s[23]) >> 5);
  a[10] = 0x1fffff & (load_3_i64(&s[26]) >> 2);
  a[11] = load_4_i64(&s[28]) >> 7;
}

/**
 * Checks if a 32-byte array represents a scalar less than the group order L.
 */
bool sc_is_valid(const uint8_t *s, size_t len) 
{
  if (len != 32) return false;
  const uint8_t L[32] = 
  {
    0xed, 0xd3, 0xf5, 0x5c, 0x1a, 0x63, 0x12, 0x58,
    0xd6, 0x9c, 0xf7, 0xa2, 0xde, 0xf9, 0xde, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,
  };
  int16_t borrow = 0;
  for (int i = 0; i < 32; i++) 
  {
    int16_t diff = (int16_t)s[i] - (int16_t)L[i] - borrow;
    borrow = (diff >> 15) & 1;
  }
  return borrow == 1;
}

/* --- SHA-512 Implementation --- */

typedef struct 
{
  uint64_t state[8];
  __uint128_t total_len;
  uint8_t buffer[128];
  size_t buffer_len;
} Sha512;

const uint64_t K512[80] = 
{
  0x428a2f98d728ae22, 0x7137449123ef65cd, 0xb5c0fbcfec4d3b2f, 0xe9b5dba58189dbbc,
  0x3956c25bf348b538, 0x59f111f1b605d019, 0x923f82a4af194f9b, 0xab1c5ed5da6d8118,
  0xd807aa98a3030242, 0x12835b0145706fbe, 0x243185be4ee4b28c, 0x550c7dc3d5ffb4e2,
  0x72be5d74f27b896f, 0x80deb1fe3b1696b1, 0x9bdc06a725c71235, 0xc19bf174cf692694,
  0xe49b69c19ef14ad2, 0xefbe4786384f25e3, 0x0fc19dc68b8cd5b5, 0x240ca1cc77ac9c65,
  0x2de92c6f592b0275, 0x4a7484aa6ea6e483, 0x5cb0a9dcbd41fbd4, 0x76f988da831153b5,
  0x983e5152ee66dfab, 0xa831c66d2db43210, 0xb00327c898fb213f, 0xbf597fc7beef0ee4,
  0xc6e00bf33da88fc2, 0xd5a79147930aa725, 0x06ca6351e003826f, 0x142929670a0e6e70,
  0x27b70a8546d22ffc, 0x2e1b21385c26c926, 0x4d2c6dfc5ac42aed, 0x53380d139d95b3df,
  0x650a73548baf63de, 0x766a0abb3c77b2a8, 0x81c2c92e47edaee6, 0x92722c851482353b,
  0xa2bfe8a14cf10364, 0xa81a664bbc423001, 0xc24b8b70d0f89791, 0xc76c51a30654be30,
  0xd192e819d6ef5218, 0xd69906245565a910, 0xf40e35855771202a, 0x106aa07032bbd1b8,
  0x19a4c116b8d2d0c8, 0x1e376c085141ab53, 0x2748774cdf8eeb99, 0x34b0bcb5e19b48a8,
  0x391c0cb3c5c95a63, 0x4ed8aa4ae3418acb, 0x5b9cca4f7763e373, 0x682e6ff3d6b2b8a3,
  0x748f82ee5defb2fc, 0x78a5636f43172f60, 0x84c87814a1f0ab72, 0x8cc702081a6439ec,
  0x90befffa23631e28, 0xa4506cebde82bde9, 0xbef9a3f7b2c67915, 0xc67178f2e372532b,
  0xca273eceea26619c, 0xd186b8c721c0c207, 0xeada7dd6cde0eb1e, 0xf57d4f7fee6ed178,
  0x06f067aa72176fba, 0x0a637dc5a2c898a6, 0x113f9804bef90dae, 0x1b710b35131c471b,
  0x28db77f523047d84, 0x32caab7b40c72493, 0x3c9ebe0a15c9bebc, 0x431d67c49c100d4c,
  0x4cc5d4becb3e42b6, 0x597f299cfc657e2a, 0x5fcb6fab3ad6faec, 0x6c44198c4a475817,
};

static uint64_t rotr64(uint64_t x, uint32_t n) 
{
  return (x >> n) | (x << (64 - n));
}

static uint64_t u64_from_be_bytes(const uint8_t b[8]) 
{
  return ((uint64_t)b[0] << 56) | ((uint64_t)b[1] << 48) | ((uint64_t)b[2] << 40) | ((uint64_t)b[3] << 32) |
        ((uint64_t)b[4] << 24) | ((uint64_t)b[5] << 16) | ((uint64_t)b[6] << 8) | (uint64_t)b[7];
}

static void u64_to_be_bytes(uint64_t v, uint8_t b[8]) 
{
  b[0] = (uint8_t)(v >> 56); b[1] = (uint8_t)(v >> 48); b[2] = (uint8_t)(v >> 40); b[3] = (uint8_t)(v >> 32);
  b[4] = (uint8_t)(v >> 24); b[5] = (uint8_t)(v >> 16); b[6] = (uint8_t)(v >> 8);  b[7] = (uint8_t)v;
}

static void u128_to_be_bytes(__uint128_t v, uint8_t b[16]) 
{
  for (int i = 0; i < 16; i++) b[i] = (uint8_t)(v >> (120 - i * 8));
}

void sha512_process_block(Sha512 *self, const uint8_t block[128]) 
{
  uint64_t w[80];

  for (int i = 0; i < 16; i++) w[i] = u64_from_be_bytes(&block[i * 8]);
  for (int i = 16; i < 80; i++) 
  {
    uint64_t s0 = rotr64(w[i - 15], 1) ^ rotr64(w[i - 15], 8) ^ (w[i - 15] >> 7);
    uint64_t s1 = rotr64(w[i - 2], 19) ^ rotr64(w[i - 2], 61) ^ (w[i - 2] >> 6);
    w[i] = w[i - 16] + s0 + w[i - 7] + s1;
  }
  uint64_t a = self->state[0];
  uint64_t b = self->state[1];
  uint64_t c = self->state[2];
  uint64_t d = self->state[3];
  uint64_t e = self->state[4];
  uint64_t f = self->state[5];
  uint64_t g = self->state[6];
  uint64_t h = self->state[7];
  for (int i = 0; i < 80; i++) 
  {
    uint64_t s1 = rotr64(e, 14) ^ rotr64(e, 18) ^ rotr64(e, 41);
    uint64_t ch = (e & f) ^ ((~e) & g);
    uint64_t temp1 = h + s1 + ch + K512[i] + w[i];
    uint64_t s0 = rotr64(a, 28) ^ rotr64(a, 34) ^ rotr64(a, 39);
    uint64_t maj = (a & b) ^ (a & c) ^ (b & c);
    uint64_t temp2 = s0 + maj;
    h = g; g = f; f = e;
    e = d + temp1;
    d = c; c = b; b = a;
    a = temp1 + temp2;
  }
  self->state[0] += a;
  self->state[1] += b;
  self->state[2] += c;
  self->state[3] += d;
  self->state[4] += e;
  self->state[5] += f;
  self->state[6] += g;
  self->state[7] += h;
}

void sha512_init(Sha512 *self) 
{
  self->state[0] = 0x6a09e667f3bcc908;
  self->state[1] = 0xbb67ae8584caa73b;
  self->state[2] = 0x3c6ef372fe94f82b;
  self->state[3] = 0xa54ff53a5f1d36f1;
  self->state[4] = 0x510e527fade682d1;
  self->state[5] = 0x9b05688c2b3e6c1f;
  self->state[6] = 0x1f83d9abfb41bd6b;
  self->state[7] = 0x5be0cd19137e2179;
  self->total_len = 0;
  memset(self->buffer, 0, 128);
  self->buffer_len = 0;
}

void sha512_update(Sha512 *self, const uint8_t *data, size_t len) 
{
  size_t offset = 0;
  if (self->buffer_len > 0) 
  {
    size_t needed = 128 - self->buffer_len;
    if (len >= needed) 
    {
      memcpy(self->buffer + self->buffer_len, data, needed);
      sha512_process_block(self, self->buffer);
      self->buffer_len = 0;
      offset = needed;
    } 
    else 
    {
      memcpy(self->buffer + self->buffer_len, data, len);
      self->buffer_len += len;
      self->total_len += len;
      return;
    }
  }
  while (offset + 128 <= len) 
  {
    sha512_process_block(self, &data[offset]);
    offset += 128;
  }
  if (offset < len) 
  {
    size_t remaining = len - offset;
    memcpy(self->buffer, &data[offset], remaining);
    self->buffer_len = remaining;
  }
  self->total_len += len;
}

void sha512_finalize(Sha512 *self, uint8_t digest[64]) 
{
  __uint128_t total_bits = self->total_len * 8;
  self->buffer[self->buffer_len] = 0x80;
  self->buffer_len += 1;
  if (self->buffer_len > 112) 
  {
    for (size_t i = self->buffer_len; i < 128; i++) self->buffer[i] = 0;
    sha512_process_block(self, self->buffer);
    self->buffer_len = 0;
  }
  for (size_t i = self->buffer_len; i < 112; i++) self->buffer[i] = 0;
  uint8_t total_bits_bytes[16];
  u128_to_be_bytes(total_bits, total_bits_bytes);
  memcpy(&self->buffer[112], total_bits_bytes, 16);
  sha512_process_block(self, self->buffer);
  for (int i = 0; i < 8; i++) u64_to_be_bytes(self->state[i], &digest[i * 8]);
}

void sha512(const uint8_t *data, size_t len, uint8_t out[64]) 
{
  Sha512 hasher;
  sha512_init(&hasher);
  sha512_update(&hasher, data, len);
  sha512_finalize(&hasher, out);
}
