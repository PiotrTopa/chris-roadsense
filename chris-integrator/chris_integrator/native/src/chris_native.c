/* chris-integrator's native core: the arithmetic and the text work of the per-message hot path.
 *
 * One C99 file, the CPython limited API (one binary for every CPython >= 3.11), no numpy headers (arrays are
 * read and written through the buffer protocol), no other dependency than libm. Every function here has a
 * pure-Python counterpart with the same contract (chris_integrator/native/__init__.py says which is in use);
 * tests/test_native.py runs both and compares them.
 *
 *   dumps(obj)                 json.dumps(obj, separators=(",", ":")): the same text, byte for byte
 *   loads(bytes | str)         json.loads: the same objects
 *   round(x, n)                builtins.round: the same result
 *   interp_all(xs, ys, qs)     trackers.view._interp_all
 *   lanes_measure, lanes_predict, lanes_fit, lanes_apply, round_lists     the lane tracker's array work
 *                              (trackers/lanes.py): the same bits as the numpy expressions
 *   ekf_predict, ekf_project, ekf_update, ekf_sym, ekf_rts     the estimator's linear algebra (core/ekf.py,
 *                              core/ego.py); elementwise steps give numpy's bits, sums of products are taken
 *                              in index order (numpy hands those to BLAS, whose order is its own)
 *
 * What is not certain to give the reference's result is never guessed: the function raises `Fallback` (or
 * calls the reference itself, `round`), and the caller takes the pure-Python path for that one call.
 * Compile with -ffp-contract=off: a fused multiply-add would change the bits.
 *
 * SPDX-License-Identifier: MIT
 */
#define Py_LIMITED_API 0x030B0000
#define PY_SSIZE_T_CLEAN
#include <Python.h>

#include <math.h>
#include <stdint.h>
#include <string.h>

#define NATIVE_VERSION 2

static PyObject *Fallback;          /* raised where the pure-Python path must decide */
static PyObject *builtin_round;
static PyObject *rounded_type;      /* a tuple subclass (values, digits): a list written rounded, see dumps */

static const double POW10[23] = {1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9, 1e10, 1e11, 1e12, 1e13, 1e14,
                                 1e15, 1e16, 1e17, 1e18, 1e19, 1e20, 1e21, 1e22};

/* ====================================================================================================
 * Text buffer
 * ==================================================================================================== */
typedef struct {
    char *p;
    size_t n, cap;
} Buf;

static int buf_room(Buf *b, size_t more)
{
    if (b->n + more <= b->cap)
        return 0;
    size_t cap = b->cap ? b->cap : 1024;
    while (cap < b->n + more) {
        if (cap > ((size_t)1 << 40)) {
            PyErr_NoMemory();
            return -1;
        }
        cap *= 2;
    }
    char *q = (char *)PyMem_Realloc(b->p, cap);
    if (q == NULL) {
        PyErr_NoMemory();
        return -1;
    }
    b->p = q;
    b->cap = cap;
    return 0;
}

static inline int buf_put(Buf *b, const char *s, size_t n)
{
    if (n == 0)                                    /* (nothing to copy; the buffer may not exist yet) */
        return 0;
    if (buf_room(b, n) < 0)
        return -1;
    memcpy(b->p + b->n, s, n);
    b->n += n;
    return 0;
}

static inline int buf_ch(Buf *b, char c)
{
    if (b->n == b->cap && buf_room(b, 1) < 0)
        return -1;
    b->p[b->n++] = c;
    return 0;
}

/* ====================================================================================================
 * JSON: writing
 * ==================================================================================================== */
static int put_u64(Buf *b, uint64_t v)
{
    char t[24];
    int i = 24;
    do {
        t[--i] = (char)('0' + v % 10);
        v /= 10;
    } while (v);
    return buf_put(b, t + i, (size_t)(24 - i));
}

/* float.__repr__ for a finite double.
 * repr is the shortest decimal string that reads back as the same double. If the double is k / 10^n for an
 * integer k below 10^15, that string is k with n decimals: decimals of 15 significant digits or fewer and
 * doubles correspond one to one, so there is no second such string and no shorter one. Everything else
 * (16 or 17 digits, or an exponent form: below 1e-4) is left to CPython's own conversion. */
static int put_float(Buf *b, double v)
{
    double a = fabs(v);
    if (a == 0.0)
        return buf_put(b, signbit(v) ? "-0.0" : "0.0", signbit(v) ? 4 : 3);
    if (a >= 1e-4 && a < 1e15) {
        for (int n = 0; n <= 19; n++) {
            double y = a * POW10[n];
            if (!(y < 1e15))
                break;
            double r = rint(y);
            /* a * 10^n is within a few units in the last place of the integer k when a is k / 10^n: only then
             * is the exact test (one division) worth making */
            if (fabs(y - r) > y * 1e-15)
                continue;
            if (r / POW10[n] == a && r >= 1.0) {
                uint64_t k = (uint64_t)r;
                if (v < 0.0 && buf_ch(b, '-') < 0)
                    return -1;
                if (n == 0) {
                    if (put_u64(b, k) < 0)
                        return -1;
                    return buf_put(b, ".0", 2);
                }
                uint64_t p = 1;
                for (int i = 0; i < n; i++)
                    p *= 10;
                if (put_u64(b, k / p) < 0 || buf_ch(b, '.') < 0)
                    return -1;
                char t[24];
                uint64_t f = k % p;
                for (int i = n - 1; i >= 0; i--) {
                    t[i] = (char)('0' + f % 10);
                    f /= 10;
                }
                return buf_put(b, t, (size_t)n);
            }
        }
    }
    char *s = PyOS_double_to_string(v, 'r', 0, Py_DTSF_ADD_DOT_0, NULL);
    if (s == NULL)
        return -1;
    int rc = buf_put(b, s, strlen(s));
    PyMem_Free(s);
    return rc;
}

static int put_double(Buf *b, double v)
{
    if (v != v)
        return buf_put(b, "NaN", 3);
    if (isinf(v))
        return v > 0 ? buf_put(b, "Infinity", 8) : buf_put(b, "-Infinity", 9);
    return put_float(b, v);
}

static int put_int(Buf *b, PyObject *o)
{
    int over = 0;
    long long v = PyLong_AsLongLongAndOverflow(o, &over);
    if (v == -1 && PyErr_Occurred())
        return -1;
    if (over) {                                    /* a big integer: int.__repr__ as json writes it */
        PyErr_SetNone(Fallback);
        return -1;
    }
    if (v < 0) {
        if (buf_ch(b, '-') < 0)
            return -1;
        return put_u64(b, (uint64_t)0 - (uint64_t)v);
    }
    return put_u64(b, (uint64_t)v);
}

/* a str as json.dumps writes it with ensure_ascii (the default): \" \\ \n \r \t \b \f, \uXXXX for every
 * other character outside 0x20..0x7e, surrogate pairs above U+FFFF */
static int put_str(Buf *b, PyObject *o)
{
    static const char HEX[] = "0123456789abcdef";
    Py_ssize_t len;
    const char *s = PyUnicode_AsUTF8AndSize(o, &len);
    if (s == NULL) {                               /* a lone surrogate: json writes it, UTF-8 cannot hold it */
        PyErr_Clear();
        PyErr_SetNone(Fallback);
        return -1;
    }
    const unsigned char *u = (const unsigned char *)s, *end = u + len;
    if (buf_room(b, (size_t)len + 2) < 0)
        return -1;
    b->p[b->n++] = '"';
    while (u < end) {
        const unsigned char *run = u;
        while (u < end && *u >= 0x20 && *u < 0x7f && *u != '"' && *u != '\\')
            u++;
        if (u > run && buf_put(b, (const char *)run, (size_t)(u - run)) < 0)
            return -1;
        if (u >= end)
            break;
        unsigned c = *u;
        char e[12];
        size_t n = 0;
        if (c < 0x80) {
            u++;
            e[n++] = '\\';
            switch (c) {
            case '"': e[n++] = '"'; break;
            case '\\': e[n++] = '\\'; break;
            case '\n': e[n++] = 'n'; break;
            case '\r': e[n++] = 'r'; break;
            case '\t': e[n++] = 't'; break;
            case '\b': e[n++] = 'b'; break;
            case '\f': e[n++] = 'f'; break;
            default:
                e[n++] = 'u'; e[n++] = '0'; e[n++] = '0';
                e[n++] = HEX[c >> 4]; e[n++] = HEX[c & 15];
            }
        } else {
            /* UTF-8 from CPython is well formed; the lengths are checked all the same */
            uint32_t cp;
            int k;
            if ((c & 0xe0) == 0xc0) { cp = c & 0x1f; k = 1; }
            else if ((c & 0xf0) == 0xe0) { cp = c & 0x0f; k = 2; }
            else if ((c & 0xf8) == 0xf0) { cp = c & 0x07; k = 3; }
            else { PyErr_SetNone(Fallback); return -1; }
            if (end - u <= k) { PyErr_SetNone(Fallback); return -1; }
            for (int i = 1; i <= k; i++)
                cp = (cp << 6) | (u[i] & 0x3f);
            u += k + 1;
            uint32_t units[2];
            int nu = 1;
            if (cp >= 0x10000) {
                cp -= 0x10000;
                units[0] = 0xd800 | (cp >> 10);
                units[1] = 0xdc00 | (cp & 0x3ff);
                nu = 2;
            } else
                units[0] = cp;
            for (int i = 0; i < nu; i++) {
                e[n++] = '\\'; e[n++] = 'u';
                e[n++] = HEX[(units[i] >> 12) & 15]; e[n++] = HEX[(units[i] >> 8) & 15];
                e[n++] = HEX[(units[i] >> 4) & 15]; e[n++] = HEX[units[i] & 15];
            }
        }
        if (buf_put(b, e, n) < 0)
            return -1;
    }
    return buf_ch(b, '"');
}

#define MAX_DEPTH 100

static int put_value(Buf *b, PyObject *o, int depth);

/* builtins.round(x, n) for a float, or -1 (with `*ok` 0) where only CPython's own rounding is certain */
static inline double round_fast(double x, int n, int *ok)
{
    double y = x * POW10[n];
    double r = rint(y);
    /* round() is correct in decimal; rint(x * 10^n) agrees wherever the product is not next to a tie (its own
     * rounding error is below 1e-7 for products below 1e9), and r / 10^n is the double nearest to that decimal */
    *ok = (fabs(y - r) < 0.5 - 1e-6) && (fabs(y) < 1e9);
    return r / POW10[n];
}

static int put_rounded(Buf *b, PyObject *o, int depth)
{
    PyObject *vals = PyTuple_GetItem(o, 0), *nd = PyTuple_GetItem(o, 1);
    if (vals == NULL || nd == NULL)
        return -1;
    long n = PyLong_AsLong(nd);
    if (n == -1 && PyErr_Occurred())
        return -1;
    if (!PyList_CheckExact(vals) || n < 0 || n > 22) {
        PyErr_SetNone(Fallback);
        return -1;
    }
    Py_ssize_t len = PyList_Size(vals);
    if (buf_ch(b, '[') < 0)
        return -1;
    for (Py_ssize_t i = 0; i < len; i++) {
        PyObject *v = PyList_GetItem(vals, i);          /* borrowed; no Python code runs while it is in use */
        if (v == NULL)
            return -1;
        if (i && buf_ch(b, ',') < 0)
            return -1;
        if (PyFloat_CheckExact(v)) {
            int ok;
            double r = round_fast(PyFloat_AsDouble(v), (int)n, &ok);
            if (ok) {
                if (put_float(b, r) < 0)
                    return -1;
                continue;
            }
        } else if (v == Py_None) {
            if (buf_put(b, "null", 4) < 0)
                return -1;
            continue;
        } else if (PyLong_CheckExact(v)) {              /* round(int, n >= 0) is the int itself */
            if (put_int(b, v) < 0)
                return -1;
            continue;
        }
        if (!PyFloat_CheckExact(v)) {                   /* any other type: the caller's pure path (its __round__ */
            PyErr_SetNone(Fallback);                    /* would run Python code in the middle of this walk) */
            return -1;
        }
        /* a float next to a decimal tie: float.__round__ itself (C code, no callbacks) */
        PyObject *r = PyObject_CallFunction(builtin_round, "Ol", v, n);
        if (r == NULL)
            return -1;
        int rc = PyFloat_CheckExact(r) ? put_double(b, PyFloat_AsDouble(r)) : -2;
        Py_DECREF(r);
        if (rc == -2)
            PyErr_SetNone(Fallback);
        if (rc < 0)
            return -1;
    }
    return buf_ch(b, ']');
}

static int put_value(Buf *b, PyObject *o, int depth)
{
    if (o == Py_None)
        return buf_put(b, "null", 4);
    if (o == Py_True)
        return buf_put(b, "true", 4);
    if (o == Py_False)
        return buf_put(b, "false", 5);
    if (PyFloat_Check(o)) {
        double v = PyFloat_AsDouble(o);
        if (v == -1.0 && PyErr_Occurred())
            return -1;
        return put_double(b, v);
    }
    if (PyUnicode_Check(o))
        return put_str(b, o);
    if (PyLong_Check(o))
        return put_int(b, o);
    if (depth >= MAX_DEPTH) {                      /* deep or circular: json.dumps says which */
        PyErr_SetNone(Fallback);
        return -1;
    }
    if (rounded_type != NULL && (PyObject *)Py_TYPE(o) == rounded_type)
        return put_rounded(b, o, depth);
    if (PyDict_CheckExact(o)) {                    /* (a subclass may order its items its own way: json asks it) */
        Py_ssize_t pos = 0;
        PyObject *k, *v;
        int first = 1;
        if (buf_ch(b, '{') < 0)
            return -1;
        /* nothing in this loop runs Python code (no callbacks, no __repr__): the dict cannot change under it */
        while (PyDict_Next(o, &pos, &k, &v)) {
            if (!PyUnicode_Check(k)) {             /* json converts int, float, bool and None keys */
                PyErr_SetNone(Fallback);
                return -1;
            }
            if (!first && buf_ch(b, ',') < 0)
                return -1;
            first = 0;
            if (put_str(b, k) < 0 || buf_ch(b, ':') < 0 || put_value(b, v, depth + 1) < 0)
                return -1;
        }
        return buf_ch(b, '}');
    }
    if (PyList_Check(o)) {
        Py_ssize_t n = PyList_Size(o);
        if (buf_ch(b, '[') < 0)
            return -1;
        for (Py_ssize_t i = 0; i < n; i++) {
            if (i && buf_ch(b, ',') < 0)
                return -1;
            PyObject *v = PyList_GetItem(o, i);
            if (v == NULL || put_value(b, v, depth + 1) < 0)
                return -1;
        }
        return buf_ch(b, ']');
    }
    if (PyTuple_Check(o)) {
        Py_ssize_t n = PyTuple_Size(o);
        if (buf_ch(b, '[') < 0)
            return -1;
        for (Py_ssize_t i = 0; i < n; i++) {
            if (i && buf_ch(b, ',') < 0)
                return -1;
            PyObject *v = PyTuple_GetItem(o, i);
            if (v == NULL || put_value(b, v, depth + 1) < 0)
                return -1;
        }
        return buf_ch(b, ']');
    }
    PyErr_SetNone(Fallback);                       /* any other type: json.dumps decides (and raises its error) */
    return -1;
}

static PyObject *n_dumps(PyObject *self, PyObject *o)
{
    Buf b = {NULL, 0, 0};
    PyObject *res = NULL;
    if (put_value(&b, o, 0) == 0)
        res = PyUnicode_DecodeLatin1(b.p, (Py_ssize_t)b.n, NULL);     /* ASCII only */
    PyMem_Free(b.p);
    return res;
}

static PyObject *n_set_rounded_type(PyObject *self, PyObject *t)
{
    if (!PyType_Check(t) || !PyType_IsSubtype((PyTypeObject *)t, &PyTuple_Type)) {
        PyErr_SetString(PyExc_TypeError, "a tuple subclass is needed");
        return NULL;
    }
    Py_INCREF(t);
    Py_XDECREF(rounded_type);
    rounded_type = t;
    Py_RETURN_NONE;
}

/* ====================================================================================================
 * JSON: reading. The grammar of json.loads (RFC 8259 plus NaN, Infinity, -Infinity). Every read is
 * bounds-checked against `end`; nesting is limited; whatever is not plain (an escape that needs
 * surrogate handling, a number beyond the exact fast path, invalid text) raises Fallback and json.loads
 * decides, so errors and corner cases are CPython's own.
 * ==================================================================================================== */
typedef struct {
    const unsigned char *p, *end;
    PyObject *memo;                 /* key text -> key object: one str per distinct key, as json does */
} Rd;

#define RD_MAX_DEPTH 64

static inline void rd_ws(Rd *r)
{
    while (r->p < r->end && (*r->p == ' ' || *r->p == '\t' || *r->p == '\n' || *r->p == '\r'))
        r->p++;
}

static PyObject *rd_fail(void)
{
    if (!PyErr_Occurred())
        PyErr_SetNone(Fallback);
    return NULL;
}

static int hex4(const unsigned char *p, uint32_t *out)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        unsigned c = p[i];
        v <<= 4;
        if (c >= '0' && c <= '9') v |= c - '0';
        else if (c >= 'a' && c <= 'f') v |= c - 'a' + 10;
        else if (c >= 'A' && c <= 'F') v |= c - 'A' + 10;
        else return -1;
    }
    *out = v;
    return 0;
}

/* a string with escapes, from its first backslash on: the text is rebuilt as UTF-8. A \uXXXX that is half of
 * a surrogate pair without its partner is left to the reference (it makes a lone surrogate). */
static PyObject *rd_string_esc(Rd *r, const unsigned char *s)
{
    Buf b = {NULL, 0, 0};
    PyObject *o = NULL;
    if (buf_put(&b, (const char *)s, (size_t)(r->p - s)) < 0)
        goto out;
    while (r->p < r->end) {
        unsigned c = *r->p;
        if (c == '"') {
            o = PyUnicode_DecodeUTF8(b.p ? b.p : "", (Py_ssize_t)b.n, "strict");
            r->p++;
            if (o == NULL)
                PyErr_Clear();
            goto out;
        }
        if (c < 0x20)
            goto out;
        if (c != '\\') {
            if (buf_ch(&b, (char)c) < 0)
                goto out;
            r->p++;
            continue;
        }
        if (r->end - r->p < 2)
            goto out;
        unsigned e = r->p[1];
        r->p += 2;
        char ch;
        switch (e) {
        case '"': ch = '"'; break;
        case '\\': ch = '\\'; break;
        case '/': ch = '/'; break;
        case 'b': ch = '\b'; break;
        case 'f': ch = '\f'; break;
        case 'n': ch = '\n'; break;
        case 'r': ch = '\r'; break;
        case 't': ch = '\t'; break;
        case 'u': {
            uint32_t cp, lo;
            if (r->end - r->p < 4 || hex4(r->p, &cp) < 0)
                goto out;
            r->p += 4;
            if (cp >= 0xd800 && cp <= 0xdbff) {
                if (r->end - r->p < 6 || r->p[0] != '\\' || r->p[1] != 'u' || hex4(r->p + 2, &lo) < 0 ||
                    lo < 0xdc00 || lo > 0xdfff)
                    goto out;
                r->p += 6;
                cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
            } else if (cp >= 0xdc00 && cp <= 0xdfff)
                goto out;
            char u[4];
            size_t n;
            if (cp < 0x80) { u[0] = (char)cp; n = 1; }
            else if (cp < 0x800) { u[0] = (char)(0xc0 | (cp >> 6)); u[1] = (char)(0x80 | (cp & 0x3f)); n = 2; }
            else if (cp < 0x10000) {
                u[0] = (char)(0xe0 | (cp >> 12)); u[1] = (char)(0x80 | ((cp >> 6) & 0x3f));
                u[2] = (char)(0x80 | (cp & 0x3f)); n = 3;
            } else {
                u[0] = (char)(0xf0 | (cp >> 18)); u[1] = (char)(0x80 | ((cp >> 12) & 0x3f));
                u[2] = (char)(0x80 | ((cp >> 6) & 0x3f)); u[3] = (char)(0x80 | (cp & 0x3f)); n = 4;
            }
            if (buf_put(&b, u, n) < 0)
                goto out;
            continue;
        }
        default:
            goto out;
        }
        if (buf_ch(&b, ch) < 0)
            goto out;
    }
out:
    PyMem_Free(b.p);
    return o ? o : rd_fail();
}

/* a string: the bytes between the quotes, which must be valid UTF-8 without control characters (json.loads
 * refuses those in its default strict mode) */
static PyObject *rd_string(Rd *r)
{
    const unsigned char *s = ++r->p;               /* past the opening quote */
    while (r->p < r->end) {
        unsigned c = *r->p;
        if (c == '"') {
            PyObject *o = PyUnicode_DecodeUTF8((const char *)s, (Py_ssize_t)(r->p - s), "strict");
            r->p++;
            if (o == NULL) {
                PyErr_Clear();
                return rd_fail();
            }
            return o;
        }
        if (c == '\\')
            return rd_string_esc(r, s);
        if (c < 0x20)
            return rd_fail();                      /* a control character: the reference names the error */
        r->p++;
    }
    return rd_fail();
}

static PyObject *rd_number(Rd *r)
{
    const unsigned char *s = r->p, *p = r->p, *end = r->end;
    int neg = 0, is_float = 0;
    if (p < end && *p == '-') {
        neg = 1;
        p++;
    }
    if (p >= end)
        return rd_fail();
    if (*p == 'I' || *p == 'N')
        return rd_fail();                          /* Infinity, NaN: rare, the reference reads them */
    uint64_t mant = 0;
    int digits = 0, frac = 0;                      /* digits read into `mant`; how many of them after the point */
    const unsigned char *ip = p;
    if (*p == '0') {
        p++;
    } else if (*p >= '1' && *p <= '9') {
        while (p < end && *p >= '0' && *p <= '9') {
            if (digits < 19)
                mant = mant * 10 + (uint64_t)(*p - '0');
            digits++;
            p++;
        }
    } else
        return rd_fail();
    int int_len = (int)(p - ip);
    if (p < end && *p == '.') {
        const unsigned char *fp = ++p;
        while (p < end && *p >= '0' && *p <= '9') {
            if (digits < 19)
                mant = mant * 10 + (uint64_t)(*p - '0');
            digits++;
            frac++;
            p++;
        }
        if (p == fp)
            return rd_fail();
        is_float = 1;
    }
    int has_exp = 0;
    if (p < end && (*p == 'e' || *p == 'E')) {
        const unsigned char *q = p + 1;
        if (q < end && (*q == '+' || *q == '-'))
            q++;
        const unsigned char *ep = q;
        while (q < end && *q >= '0' && *q <= '9')
            q++;
        if (q == ep)
            return rd_fail();
        p = q;
        has_exp = is_float = 1;
    }
    if (!is_float) {
        /* up to 19 digits fit 64 bits unsigned; the value must fit a signed 64-bit integer */
        if (int_len > 19 || mant > (uint64_t)INT64_MAX)
            return rd_fail();                      /* a big integer: the reference reads it */
        r->p = p;
        long long v = (long long)mant;
        return PyLong_FromLongLong(neg ? -v : v);
    }
    /* mant / 10^frac with mant <= 2^53 and frac <= 22: both are exact doubles, and one correctly rounded
     * division gives the double nearest to the decimal, which is what CPython's strtod returns */
    if (!has_exp && digits <= 19 && frac <= 22 && mant <= ((uint64_t)1 << 53)) {
        r->p = p;
        double v = (double)mant / POW10[frac];
        return PyFloat_FromDouble(neg ? -v : v);
    }
    /* any other float (an exponent, 17 digits): CPython's own conversion of the token, as json does */
    char tok[64];
    size_t n = (size_t)(p - s);
    if (n >= sizeof tok)
        return rd_fail();
    memcpy(tok, s, n);
    tok[n] = 0;
    char *stop = NULL;
    double v = PyOS_string_to_double(tok, &stop, NULL);
    if (v == -1.0 && PyErr_Occurred()) {
        PyErr_Clear();
        return rd_fail();
    }
    if (stop != tok + n)
        return rd_fail();
    r->p = p;
    return PyFloat_FromDouble(v);
}

static PyObject *rd_value(Rd *r, int depth)
{
    rd_ws(r);
    if (r->p >= r->end)
        return rd_fail();
    unsigned c = *r->p;
    if (c == '"')
        return rd_string(r);
    if (c == '{') {
        if (depth >= RD_MAX_DEPTH)
            return rd_fail();
        PyObject *d = PyDict_New();
        if (d == NULL)
            return NULL;
        r->p++;
        rd_ws(r);
        if (r->p < r->end && *r->p == '}') {
            r->p++;
            return d;
        }
        for (;;) {
            rd_ws(r);
            if (r->p >= r->end || *r->p != '"')
                goto bad_d;
            /* the key: looked up in the memo by its bytes, so equal keys are one object */
            PyObject *k = rd_string(r);
            if (k == NULL)
                goto bad_d;
            PyObject *kk = PyDict_GetItemWithError(r->memo, k);   /* borrowed */
            if (kk == NULL) {
                if (PyErr_Occurred() || PyDict_SetItem(r->memo, k, k) < 0) {
                    Py_DECREF(k);
                    goto bad_d;
                }
                kk = k;
            }
            Py_INCREF(kk);
            Py_DECREF(k);
            rd_ws(r);
            if (r->p >= r->end || *r->p != ':') {
                Py_DECREF(kk);
                goto bad_d;
            }
            r->p++;
            PyObject *v = rd_value(r, depth + 1);
            if (v == NULL) {
                Py_DECREF(kk);
                goto bad_d;
            }
            int rc = PyDict_SetItem(d, kk, v);
            Py_DECREF(kk);
            Py_DECREF(v);
            if (rc < 0)
                goto bad_d;
            rd_ws(r);
            if (r->p >= r->end)
                goto bad_d;
            if (*r->p == ',') {
                r->p++;
                continue;
            }
            if (*r->p == '}') {
                r->p++;
                return d;
            }
            goto bad_d;
        }
    bad_d:
        Py_DECREF(d);
        return rd_fail();
    }
    if (c == '[') {
        if (depth >= RD_MAX_DEPTH)
            return rd_fail();
        PyObject *l = PyList_New(0);
        if (l == NULL)
            return NULL;
        r->p++;
        rd_ws(r);
        if (r->p < r->end && *r->p == ']') {
            r->p++;
            return l;
        }
        for (;;) {
            PyObject *v = rd_value(r, depth + 1);
            if (v == NULL)
                goto bad_l;
            int rc = PyList_Append(l, v);
            Py_DECREF(v);
            if (rc < 0)
                goto bad_l;
            rd_ws(r);
            if (r->p >= r->end)
                goto bad_l;
            if (*r->p == ',') {
                r->p++;
                continue;
            }
            if (*r->p == ']') {
                r->p++;
                return l;
            }
            goto bad_l;
        }
    bad_l:
        Py_DECREF(l);
        return rd_fail();
    }
    if (c == '-' || (c >= '0' && c <= '9'))
        return rd_number(r);
    size_t left = (size_t)(r->end - r->p);
    if (c == 't' && left >= 4 && memcmp(r->p, "true", 4) == 0) {
        r->p += 4;
        Py_RETURN_TRUE;
    }
    if (c == 'f' && left >= 5 && memcmp(r->p, "false", 5) == 0) {
        r->p += 5;
        Py_RETURN_FALSE;
    }
    if (c == 'n' && left >= 4 && memcmp(r->p, "null", 4) == 0) {
        r->p += 4;
        Py_RETURN_NONE;
    }
    return rd_fail();
}

static PyObject *n_loads(PyObject *self, PyObject *arg)
{
    const char *s;
    Py_ssize_t len;
    Py_buffer view;
    int have_view = 0;
    if (PyUnicode_Check(arg)) {
        s = PyUnicode_AsUTF8AndSize(arg, &len);
        if (s == NULL) {
            PyErr_Clear();
            PyErr_SetNone(Fallback);
            return NULL;
        }
    } else {
        if (!PyBytes_Check(arg) && !PyByteArray_Check(arg)) {     /* json.loads takes str, bytes, bytearray */
            PyErr_SetNone(Fallback);
            return NULL;
        }
        if (PyObject_GetBuffer(arg, &view, PyBUF_SIMPLE) < 0) {
            PyErr_Clear();
            PyErr_SetNone(Fallback);
            return NULL;
        }
        have_view = 1;
        s = (const char *)view.buf;
        len = view.len;
    }
    Rd r;
    r.p = (const unsigned char *)s;
    r.end = r.p + len;
    r.memo = PyDict_New();
    PyObject *o = NULL;
    if (r.memo != NULL) {
        o = rd_value(&r, 0);
        if (o != NULL) {
            rd_ws(&r);
            if (r.p != r.end) {                    /* text after the value: the reference names the error */
                Py_DECREF(o);
                o = NULL;
                PyErr_SetNone(Fallback);
            }
        }
        Py_DECREF(r.memo);
    }
    if (have_view)
        PyBuffer_Release(&view);
    return o;
}

/* ====================================================================================================
 * round
 * ==================================================================================================== */
static PyObject *n_round(PyObject *self, PyObject *args)
{
    Py_ssize_t na = PyTuple_Size(args);
    if (na == 2) {
        PyObject *x = PyTuple_GetItem(args, 0), *nd = PyTuple_GetItem(args, 1);
        if (PyFloat_CheckExact(x) && PyLong_CheckExact(nd)) {
            int over = 0;
            long n = PyLong_AsLongAndOverflow(nd, &over);
            if (!over && n >= 0 && n <= 22) {
                int ok;
                double r = round_fast(PyFloat_AsDouble(x), (int)n, &ok);
                if (ok)
                    return PyFloat_FromDouble(r);
            }
        }
    }
    return PyObject_CallObject(builtin_round, args);
}

/* round_list(values, n): [v if v is None else round(v, n) for v in values] for a list */
static PyObject *n_round_list(PyObject *self, PyObject *args)
{
    PyObject *vals;
    long n;
    if (!PyArg_ParseTuple(args, "Ol", &vals, &n))
        return NULL;
    if (!PyList_CheckExact(vals)) {
        PyErr_SetString(PyExc_TypeError, "round_list: a list is needed");
        return NULL;
    }
    Py_ssize_t len = PyList_Size(vals);
    PyObject *out = PyList_New(len);
    if (out == NULL)
        return NULL;
    for (Py_ssize_t i = 0; i < len; i++) {
        PyObject *v = PyList_GetItem(vals, i), *r = NULL;
        if (v == NULL)
            goto fail;
        if (v == Py_None) {
            r = v;
            Py_INCREF(r);
        } else if (PyFloat_CheckExact(v) && n >= 0 && n <= 22) {
            int ok;
            double d = round_fast(PyFloat_AsDouble(v), (int)n, &ok);
            if (ok)
                r = PyFloat_FromDouble(d);
        }
        if (r == NULL && !PyErr_Occurred()) {
            Py_INCREF(v);
            r = PyObject_CallFunction(builtin_round, "Ol", v, n);
            Py_DECREF(v);
        }
        if (r == NULL)
            goto fail;
        PyList_SetItem(out, i, r);
    }
    return out;
fail:
    Py_DECREF(out);
    return NULL;
}

/* ====================================================================================================
 * Arrays through the buffer protocol
 * ==================================================================================================== */
typedef struct {
    Py_buffer b;
    int held;
} View;

static int view_get(PyObject *o, View *v, int writable, char kind, int ndim, const char *what)
{
    v->held = 0;
    if (PyObject_GetBuffer(o, &v->b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT | (writable ? PyBUF_WRITABLE : 0)) < 0)
        return -1;
    v->held = 1;
    const char *f = v->b.format ? v->b.format : "B";
    if (*f == '@' || *f == '=' || *f == '<')
        f++;
    int ok = 0;
    if (kind == 'd')
        ok = (f[0] == 'd' && f[1] == 0 && v->b.itemsize == 8);
    else if (kind == 'b')                           /* a bool array */
        ok = (f[0] == '?' && f[1] == 0 && v->b.itemsize == 1);
    else                                            /* an index array: the platform's signed 64-bit integer */
        ok = ((f[0] == 'l' || f[0] == 'q' || f[0] == 'n') && f[1] == 0 && v->b.itemsize == 8);
    if (!ok || v->b.ndim != ndim) {
        PyBuffer_Release(&v->b);
        v->held = 0;
        PyErr_Format(PyExc_TypeError, "%s: a C-contiguous %s array of %d dimension(s) is needed", what,
                     kind == 'd' ? "float64" : kind == 'b' ? "bool" : "int64", ndim);
        return -1;
    }
    return 0;
}

static void view_drop(View *v)
{
    if (v->held) {
        PyBuffer_Release(&v->b);
        v->held = 0;
    }
}

#define DATA(v) ((double *)(v).b.buf)

/* ====================================================================================================
 * The estimator's linear algebra (core/ekf.py, core/ego.py are the reference; names follow them)
 * ==================================================================================================== */
enum { PE, PN, PSI, V, A, W, AL, Z, TH, SV, NCORE };
#define PAR0 13                  /* scalars at the head of the model's parameter block */
#define PI 3.141592653589793     /* math.pi */

/* out[:NCORE, :] = Fc @ M[:NCORE, :], the other rows of M as they are; terms whose coefficient is exactly
 * zero are skipped (the kinematic block is mostly zeros), the others are summed in index order */
static void fc_rows(const double *Fc, const double *M, double *out, Py_ssize_t n)
{
    for (int i = 0; i < NCORE; i++) {
        double *o = out + (Py_ssize_t)i * n;
        int first = 1;
        for (int k = 0; k < NCORE; k++) {
            double f = Fc[i * NCORE + k];
            if (f == 0.0)
                continue;
            const double *m = M + (Py_ssize_t)k * n;
            if (first) {
                for (Py_ssize_t j = 0; j < n; j++)
                    o[j] = f * m[j];
                first = 0;
            } else {
                for (Py_ssize_t j = 0; j < n; j++)
                    o[j] += f * m[j];
            }
        }
        if (first)
            for (Py_ssize_t j = 0; j < n; j++)
                o[j] = 0.0;
    }
    if (out != M)
        memcpy(out + (Py_ssize_t)NCORE * n, M + (Py_ssize_t)NCORE * n, (size_t)((n - NCORE) * n) * sizeof(double));
}

/* M[:, :NCORE] = M[:, :NCORE] @ Fc.T, in place */
static void fc_cols(const double *Fc, double *M, Py_ssize_t n)
{
    double t[NCORE];
    for (Py_ssize_t r = 0; r < n; r++) {
        double *m = M + r * n;
        for (int j = 0; j < NCORE; j++) {
            double acc = 0.0;
            int first = 1;
            for (int k = 0; k < NCORE; k++) {
                double f = Fc[j * NCORE + k];
                if (f == 0.0)
                    continue;
                if (first) {
                    acc = m[k] * f;
                    first = 0;
                } else
                    acc += m[k] * f;
            }
            t[j] = acc;
        }
        for (int j = 0; j < NCORE; j++)
            m[j] = t[j];
    }
}

/* ekf_predict(x, P, dt, par, rho_out | None, x_out, P_out, Fc_out)
 * par: [jerk_psd, yaw_jerk_psd, pos_psd, alt_psd, grade_psd, scale_psd, the five caps of ego._CAPS (V, A, W,
 *       AL, PSI), n_clamp, n_gm, psd of the aux states ..., (index, lo, hi) of every clamp ... (NaN: no bound),
 *       (index, tau, sigma) of every Gauss-Markov state ...]   (EgoModel._native_par)
 * rho_out: with Gauss-Markov states, the array that receives their decay over dt (1 for every other state):
 * the diagonal part of the transition, which the smoother needs. */
static PyObject *n_ekf_predict(PyObject *self, PyObject *args)
{
    PyObject *ox, *oP, *opar, *orr, *oxo, *oPo, *oF;
    double dt;
    View x, P, par, rr, xo, Po, F;
    x.held = P.held = par.held = rr.held = xo.held = Po.held = F.held = 0;
    double *gq = NULL;
    PyObject *res = NULL;
    if (!PyArg_ParseTuple(args, "OOdOOOOO", &ox, &oP, &dt, &opar, &orr, &oxo, &oPo, &oF))
        return NULL;
    if (view_get(ox, &x, 0, 'd', 1, "x") < 0 || view_get(oP, &P, 0, 'd', 2, "P") < 0 ||
        view_get(opar, &par, 0, 'd', 1, "par") < 0 || view_get(oxo, &xo, 1, 'd', 1, "x_out") < 0 ||
        view_get(oPo, &Po, 1, 'd', 2, "P_out") < 0 || view_get(oF, &F, 1, 'd', 2, "Fc_out") < 0)
        goto done;
    int has_rho = (orr != Py_None);
    if (has_rho && view_get(orr, &rr, 1, 'd', 1, "rho_out") < 0)
        goto done;
    Py_ssize_t n = x.b.shape[0];
    Py_ssize_t npar = par.b.shape[0];
    if (n < NCORE || P.b.shape[0] != n || P.b.shape[1] != n || xo.b.shape[0] != n || Po.b.shape[0] != n ||
        Po.b.shape[1] != n || F.b.shape[0] != NCORE || F.b.shape[1] != NCORE || npar < PAR0 ||
        (has_rho && rr.b.shape[0] != n) || DATA(Po) == DATA(P) || DATA(xo) == DATA(x)) {
        PyErr_SetString(PyExc_ValueError, "ekf_predict: shapes");
        goto done;
    }
    const double *c = DATA(par);
    Py_ssize_t nclamp = (Py_ssize_t)c[11], ngm = (Py_ssize_t)c[12];
    if (nclamp < 0 || ngm < 0 || npar != PAR0 + (n - NCORE) + 3 * nclamp + 3 * ngm || has_rho != (ngm > 0)) {
        PyErr_SetString(PyExc_ValueError, "ekf_predict: parameter block");
        goto done;
    }
    const double *psd = c + PAR0, *clamp = c + PAR0 + (n - NCORE), *gm = clamp + 3 * nclamp;
    if (has_rho) {
        /* Gauss-Markov states: decay r = exp(-dt / tau), added variance q = sigma^2 (1 - exp(-2 dt / tau)) */
        double *r = DATA(rr);
        gq = (double *)PyMem_Calloc((size_t)n, sizeof(double));
        if (gq == NULL) {
            PyErr_NoMemory();
            goto done;
        }
        for (Py_ssize_t i = 0; i < n; i++)
            r[i] = 1.0;
        for (Py_ssize_t q = 0; q < ngm; q++) {
            Py_ssize_t i = (Py_ssize_t)gm[3 * q];
            double tau = gm[3 * q + 1], sig = gm[3 * q + 2];
            if (i < 0 || i >= n) {
                PyErr_SetString(PyExc_ValueError, "ekf_predict: Gauss-Markov index");
                goto done;
            }
            r[i] = exp(-dt / tau);
            gq[i] = sig * sig * (1.0 - exp(-2.0 * dt / tau));
        }
    }
    const double *xi = DATA(x);
    double *xn = DATA(xo), *Pn = DATA(Po), *Fc = DATA(F);

    memcpy(xn, xi, (size_t)n * sizeof(double));
    double v = xi[V], a = xi[A], w = xi[W], al = xi[AL], psi = xi[PSI], th = xi[TH];
    double dt2 = dt * dt;
    double g = 1.0 / (1.0 + xi[SV]);
    double ds = (v * dt + 0.5 * a * dt2) * g;
    double pm = psi + 0.5 * w * dt + al * dt2 / 6.0;
    double cm = cos(pm), sm = sin(pm);
    double sth = sin(th), cth = cos(th);
    xn[PE] += ds * cm;
    xn[PN] += ds * sm;
    xn[PSI] = psi + w * dt + 0.5 * al * dt2;
    xn[V] = v + a * dt;
    xn[W] = w + al * dt;
    xn[Z] += ds * sth;

    double dsm = -ds * sm, dcm = ds * cm, h = 0.5 * dt;
    memset(Fc, 0, NCORE * NCORE * sizeof(double));
#define FC(i, j) Fc[(i) * NCORE + (j)]
    for (int i = 0; i < NCORE; i++)
        FC(i, i) = 1.0;
    FC(0, 2) = dsm; FC(0, 3) = dt * cm * g; FC(0, 4) = 0.5 * dt2 * cm * g; FC(0, 5) = dsm * h;
    FC(0, 6) = dsm * dt2 / 6.0; FC(0, 9) = -ds * g * cm;
    FC(1, 2) = dcm; FC(1, 3) = dt * sm * g; FC(1, 4) = 0.5 * dt2 * sm * g; FC(1, 5) = dcm * h;
    FC(1, 6) = dcm * dt2 / 6.0; FC(1, 9) = -ds * g * sm;
    FC(2, 5) = dt; FC(2, 6) = 0.5 * dt2;
    FC(3, 4) = dt;
    FC(5, 6) = dt;
    FC(7, 3) = dt * sth * g; FC(7, 4) = 0.5 * dt2 * sth * g; FC(7, 8) = ds * cth; FC(7, 9) = -ds * g * sth;
#undef FC
    if (has_rho) {
        const double *r = DATA(rr);
        for (Py_ssize_t i = 0; i < n; i++)
            xn[i] *= r[i];
    }
    fc_rows(Fc, DATA(P), Pn, n);
    fc_cols(Fc, Pn, n);
#define PP(i, j) Pn[(Py_ssize_t)(i) * n + (j)]
    if (has_rho) {
        const double *r = DATA(rr);
        for (Py_ssize_t i = 0; i < n; i++)
            for (Py_ssize_t j = 0; j < n; j++)
                PP(i, j) *= r[i];
        for (Py_ssize_t i = 0; i < n; i++)
            for (Py_ssize_t j = 0; j < n; j++)
                PP(i, j) *= r[j];
    }
    /* process noise (white jerk and yaw jerk integrated over dt; position noise along the heading) */
    double dt3 = dt2 * dt, dt4 = dt2 * dt2, dt5 = dt2 * dt2 * dt;
    double qj = c[0], qy = c[1];
    double q55 = qj * dt5 / 20.0, q44 = qj * dt4 / 8.0, q33 = qj * dt3 / 6.0;
    double pp = c[2] * dt;
    double Qk[7][7] = {
        {q55 * cm * cm + pp, q55 * cm * sm, 0.0, q44 * cm, q33 * cm, 0.0, 0.0},
        {q55 * cm * sm, q55 * sm * sm + pp, 0.0, q44 * sm, q33 * sm, 0.0, 0.0},
        {0.0, 0.0, qy * dt5 / 20.0, 0.0, 0.0, qy * dt4 / 8.0, qy * dt3 / 6.0},
        {q44 * cm, q44 * sm, 0.0, qj * dt3 / 3.0, qj * dt2 / 2.0, 0.0, 0.0},
        {q33 * cm, q33 * sm, 0.0, qj * dt2 / 2.0, qj * dt, 0.0, 0.0},
        {0.0, 0.0, qy * dt4 / 8.0, 0.0, 0.0, qy * dt3 / 3.0, qy * dt2 / 2.0},
        {0.0, 0.0, qy * dt3 / 6.0, 0.0, 0.0, qy * dt2 / 2.0, qy * dt}};
    for (int i = 0; i < 7; i++)
        for (int j = 0; j < 7; j++)
            PP(i, j) += Qk[i][j];
    PP(Z, Z) += c[3] * dt;
    PP(TH, TH) += c[4] * dt;
    PP(SV, SV) += c[5] * dt;
    for (Py_ssize_t i = NCORE; i < n; i++)
        PP(i, i) += psd[i - NCORE] * dt;
    if (has_rho) {
        for (Py_ssize_t i = NCORE; i < n; i++)
            PP(i, i) += gq[i];
    }
    if (dt > 0.5) {
        /* a gap without data: bounded quantities keep a bounded uncertainty (ego._CAPS) */
        static const int cap_i[5] = {V, A, W, AL, PSI};
        const double *cap_lim = c + 6;
        for (int q = 0; q < 5; q++) {
            int i = cap_i[q];
            if (PP(i, i) > cap_lim[q]) {
                double k = sqrt(cap_lim[q] / PP(i, i));
                for (Py_ssize_t j = 0; j < n; j++)
                    PP(i, j) *= k;
                for (Py_ssize_t j = 0; j < n; j++)
                    PP(j, i) *= k;
            }
        }
    }
#undef PP
    for (Py_ssize_t q = 0; q < nclamp; q++) {
        Py_ssize_t i = (Py_ssize_t)clamp[3 * q];
        double lo = clamp[3 * q + 1], hi = clamp[3 * q + 2];
        if (i < 0 || i >= n)
            continue;
        if (lo == lo && xn[i] < lo)
            xn[i] = lo;
        else if (hi == hi && xn[i] > hi)
            xn[i] = hi;
    }
    res = Py_None;
    Py_INCREF(res);
done:
    PyMem_Free(gq);
    view_drop(&x); view_drop(&P); view_drop(&par); view_drop(&rr);
    view_drop(&xo); view_drop(&Po); view_drop(&F);
    return res;
}

/* ekf_project(P, idx, coef, ph_out) -> s0:   ph = P[:, idx] @ coef;  s0 = coef @ ph[idx] */
static PyObject *n_ekf_project(PyObject *self, PyObject *args)
{
    PyObject *oP, *oi, *oc, *oph;
    View P, idx, coef, ph;
    P.held = idx.held = coef.held = ph.held = 0;
    PyObject *res = NULL;
    if (!PyArg_ParseTuple(args, "OOOO", &oP, &oi, &oc, &oph))
        return NULL;
    if (view_get(oP, &P, 0, 'd', 2, "P") < 0 || view_get(oi, &idx, 0, 'i', 1, "idx") < 0 ||
        view_get(oc, &coef, 0, 'd', 1, "coef") < 0 || view_get(oph, &ph, 1, 'd', 1, "ph") < 0)
        goto done;
    Py_ssize_t n = P.b.shape[0], m = idx.b.shape[0];
    const int64_t *ix = (const int64_t *)idx.b.buf;
    if (P.b.shape[1] != n || coef.b.shape[0] != m || ph.b.shape[0] != n || m < 1) {
        PyErr_SetString(PyExc_ValueError, "ekf_project: shapes");
        goto done;
    }
    for (Py_ssize_t j = 0; j < m; j++)
        if (ix[j] < 0 || ix[j] >= n) {
            PyErr_SetString(PyExc_ValueError, "ekf_project: index");
            goto done;
        }
    const double *p = DATA(P), *cf = DATA(coef);
    double *o = DATA(ph);
    for (Py_ssize_t i = 0; i < n; i++) {
        const double *row = p + i * n;
        double acc = row[ix[0]] * cf[0];
        for (Py_ssize_t j = 1; j < m; j++)
            acc += row[ix[j]] * cf[j];
        o[i] = acc;
    }
    double s0 = cf[0] * o[ix[0]];
    for (Py_ssize_t j = 1; j < m; j++)
        s0 += cf[j] * o[ix[j]];
    res = PyFloat_FromDouble(s0);
done:
    view_drop(&P); view_drop(&idx); view_drop(&coef); view_drop(&ph);
    return res;
}

/* ekf_update(x, P, ph, s, nu, freeze | None): one scalar row applied in place.
 *   k = ph / s;  x += k * nu;  P = P - k[:, None] * ph[None, :]
 * with `freeze` (Schmidt-Kalman: those states are not corrected), the Joseph form:
 *   k[freeze] = 0;  P = P - kp - kp.T + s * (k[:, None] * k[None, :]);  P = 0.5 * (P + P.T)   (kp = k ph^T) */
static PyObject *n_ekf_update(PyObject *self, PyObject *args)
{
    PyObject *ox, *oP, *oph, *ofz;
    double s, nu;
    View x, P, ph, fz;
    x.held = P.held = ph.held = fz.held = 0;
    PyObject *res = NULL;
    double *k = NULL;
    if (!PyArg_ParseTuple(args, "OOOddO", &ox, &oP, &oph, &s, &nu, &ofz))
        return NULL;
    if (view_get(ox, &x, 1, 'd', 1, "x") < 0 || view_get(oP, &P, 1, 'd', 2, "P") < 0 ||
        view_get(oph, &ph, 0, 'd', 1, "ph") < 0)
        goto done;
    int frozen = (ofz != Py_None);
    if (frozen && view_get(ofz, &fz, 0, 'i', 1, "freeze") < 0)
        goto done;
    Py_ssize_t n = x.b.shape[0];
    if (P.b.shape[0] != n || P.b.shape[1] != n || ph.b.shape[0] != n) {
        PyErr_SetString(PyExc_ValueError, "ekf_update: shapes");
        goto done;
    }
    k = (double *)PyMem_Malloc((size_t)(n ? n : 1) * sizeof(double));
    if (k == NULL) {
        PyErr_NoMemory();
        goto done;
    }
    const double *h = DATA(ph);
    double *xx = DATA(x), *p = DATA(P);
    for (Py_ssize_t i = 0; i < n; i++)
        k[i] = h[i] / s;
    if (frozen) {
        const int64_t *f = (const int64_t *)fz.b.buf;
        Py_ssize_t m = fz.b.shape[0];
        for (Py_ssize_t j = 0; j < m; j++) {
            if (f[j] < 0 || f[j] >= n) {
                PyErr_SetString(PyExc_ValueError, "ekf_update: freeze index");
                goto done;
            }
        }
        for (Py_ssize_t j = 0; j < m; j++)
            k[f[j]] = 0.0;
    }
    for (Py_ssize_t i = 0; i < n; i++)
        xx[i] += k[i] * nu;
    if (!frozen) {
        for (Py_ssize_t i = 0; i < n; i++) {
            double ki = k[i];
            double *row = p + i * n;
            for (Py_ssize_t j = 0; j < n; j++)
                row[j] = row[j] - ki * h[j];
        }
    } else {
        /* ((P - kp) - kp.T) + s * (k k^T), then the symmetric part; pairs (i, j), (j, i) are taken together
         * so that the in-place result is the one of the out-of-place expression */
        for (Py_ssize_t i = 0; i < n; i++) {
            for (Py_ssize_t j = i; j < n; j++) {
                double a = ((p[i * n + j] - k[i] * h[j]) - k[j] * h[i]) + s * (k[i] * k[j]);
                double b = ((p[j * n + i] - k[j] * h[i]) - k[i] * h[j]) + s * (k[j] * k[i]);
                p[i * n + j] = 0.5 * (a + b);
                p[j * n + i] = 0.5 * (b + a);
            }
        }
    }
    res = Py_None;
    Py_INCREF(res);
done:
    PyMem_Free(k);
    view_drop(&x); view_drop(&P); view_drop(&ph); view_drop(&fz);
    return res;
}

/* ekf_sym(P): P = 0.5 * (P + P.T) in place */
static PyObject *n_ekf_sym(PyObject *self, PyObject *oP)
{
    View P;
    if (view_get(oP, &P, 1, 'd', 2, "P") < 0)
        return NULL;
    Py_ssize_t n = P.b.shape[0];
    if (P.b.shape[1] != n) {
        view_drop(&P);
        PyErr_SetString(PyExc_ValueError, "ekf_sym: shape");
        return NULL;
    }
    double *p = DATA(P);
    for (Py_ssize_t i = 0; i < n; i++)
        for (Py_ssize_t j = i; j < n; j++) {
            double a = p[i * n + j], b = p[j * n + i];
            p[i * n + j] = 0.5 * (a + b);
            p[j * n + i] = 0.5 * (b + a);
        }
    view_drop(&P);
    Py_RETURN_NONE;
}

/* LU with partial pivoting of A (n x n, overwritten), then the solution of A X = B for the n columns of B
 * (overwritten). Returns -1 on an exactly zero pivot (numpy: LinAlgError). */
static int lu_solve(double *A, double *B, Py_ssize_t n, Py_ssize_t *piv)
{
    for (Py_ssize_t c = 0; c < n; c++) {
        Py_ssize_t best = c;
        double bv = fabs(A[c * n + c]);
        for (Py_ssize_t r = c + 1; r < n; r++) {
            double t = fabs(A[r * n + c]);
            if (t > bv) {
                bv = t;
                best = r;
            }
        }
        if (!(bv > 0.0))                            /* zero or NaN */
            return -1;
        piv[c] = best;
        if (best != c) {
            for (Py_ssize_t j = 0; j < n; j++) {
                double t = A[c * n + j]; A[c * n + j] = A[best * n + j]; A[best * n + j] = t;
                t = B[c * n + j]; B[c * n + j] = B[best * n + j]; B[best * n + j] = t;
            }
        }
        double d = A[c * n + c];
        for (Py_ssize_t r = c + 1; r < n; r++) {
            double f = A[r * n + c] / d;
            if (f == 0.0)
                continue;
            A[r * n + c] = f;
            for (Py_ssize_t j = c + 1; j < n; j++)
                A[r * n + j] -= f * A[c * n + j];
            for (Py_ssize_t j = 0; j < n; j++)
                B[r * n + j] -= f * B[c * n + j];
        }
    }
    for (Py_ssize_t r = n - 1; r >= 0; r--) {
        double d = A[r * n + r];
        for (Py_ssize_t j = 0; j < n; j++) {
            double acc = B[r * n + j];
            for (Py_ssize_t q = r + 1; q < n; q++)
                acc -= A[r * n + q] * B[q * n + j];
            B[r * n + j] = acc / d;
        }
    }
    return 0;
}

static double wrap_pi(double a)
{
    /* Python: (a + pi) % (2 pi) - pi, the remainder with the divisor's sign */
    double b = 2.0 * PI;
    double m = fmod(a + PI, b);
    if (m != 0.0) {
        if (m < 0.0)
            m += b;
    } else
        m = 0.0;
    return m - PI;
}

/* ekf_rts(Pp, Fc, rho | None, Pa, xa, xp, xs, Ps, wrap, xs_out, Ps_out) -> bool
 * One Rauch-Tung-Striebel step back over a prediction (core/ekf.py `at`):
 *   C = solve(Pp, F @ Pa).T;  xs_out = xa + C @ diff(xs, xp);  Ps_out = Pa + C @ (Ps - Pp) @ C.T
 * F is the model's transition (the kinematic block Fc and the diagonal rho, ego.apply_F); `wrap`: a tuple of
 * angle state indices whose difference is wrapped. False: Pp is singular (the caller keeps the filtered state). */
static PyObject *n_ekf_rts(PyObject *self, PyObject *args)
{
    PyObject *oPp, *oF, *orho, *oPa, *oxa, *oxp, *oxs, *oPs, *owrap, *oxo, *oPo;
    View Pp, F, rho, Pa, xa, xp, xs, Ps, xo, Po;
    Pp.held = F.held = rho.held = Pa.held = xa.held = xp.held = xs.held = Ps.held = xo.held = Po.held = 0;
    PyObject *res = NULL;
    double *work = NULL;
    Py_ssize_t *piv = NULL;
    if (!PyArg_ParseTuple(args, "OOOOOOOOOOO", &oPp, &oF, &orho, &oPa, &oxa, &oxp, &oxs, &oPs, &owrap, &oxo, &oPo))
        return NULL;
    if (view_get(oPp, &Pp, 0, 'd', 2, "Pp") < 0 || view_get(oF, &F, 0, 'd', 2, "Fc") < 0 ||
        view_get(oPa, &Pa, 0, 'd', 2, "Pa") < 0 || view_get(oxa, &xa, 0, 'd', 1, "xa") < 0 ||
        view_get(oxp, &xp, 0, 'd', 1, "xp") < 0 || view_get(oxs, &xs, 0, 'd', 1, "xs") < 0 ||
        view_get(oPs, &Ps, 0, 'd', 2, "Ps") < 0 || view_get(oxo, &xo, 1, 'd', 1, "xs_out") < 0 ||
        view_get(oPo, &Po, 1, 'd', 2, "Ps_out") < 0)
        goto done;
    int has_rho = (orho != Py_None);
    if (has_rho && view_get(orho, &rho, 0, 'd', 1, "rho") < 0)
        goto done;
    Py_ssize_t n = xa.b.shape[0];
    if (n < NCORE || Pp.b.shape[0] != n || Pp.b.shape[1] != n || Pa.b.shape[0] != n || Pa.b.shape[1] != n ||
        Ps.b.shape[0] != n || Ps.b.shape[1] != n || Po.b.shape[0] != n || Po.b.shape[1] != n ||
        xp.b.shape[0] != n || xs.b.shape[0] != n || xo.b.shape[0] != n || F.b.shape[0] != NCORE ||
        F.b.shape[1] != NCORE || (has_rho && rho.b.shape[0] != n) || !PyTuple_Check(owrap)) {
        PyErr_SetString(PyExc_ValueError, "ekf_rts: shapes");
        goto done;
    }
    size_t nn = (size_t)n * (size_t)n;
    work = (double *)PyMem_Malloc((4 * nn + (size_t)n) * sizeof(double));
    piv = (Py_ssize_t *)PyMem_Malloc((size_t)n * sizeof(Py_ssize_t));
    if (work == NULL || piv == NULL) {
        PyErr_NoMemory();
        goto done;
    }
    double *Aw = work, *X = work + nn, *D = work + 2 * nn, *T = work + 3 * nn, *d = work + 4 * nn;
    memcpy(Aw, DATA(Pp), nn * sizeof(double));
    fc_rows(DATA(F), DATA(Pa), X, n);                       /* X = F @ Pa */
    if (has_rho) {
        const double *r = DATA(rho);
        for (Py_ssize_t i = 0; i < n; i++)
            for (Py_ssize_t j = 0; j < n; j++)
                X[i * n + j] *= r[i];
    }
    if (lu_solve(Aw, X, n, piv) < 0) {
        res = Py_False;
        Py_INCREF(res);
        goto done;
    }
    /* C = X.T:  C[i][j] = X[j][i] */
    const double *vxs = DATA(xs), *vxp = DATA(xp), *vxa = DATA(xa), *vPs = DATA(Ps), *vPp = DATA(Pp), *vPa = DATA(Pa);
    for (Py_ssize_t i = 0; i < n; i++)
        d[i] = vxs[i] - vxp[i];
    Py_ssize_t nw = PyTuple_Size(owrap);
    for (Py_ssize_t q = 0; q < nw; q++) {
        Py_ssize_t i = PyLong_AsSsize_t(PyTuple_GetItem(owrap, q));
        if (i == -1 && PyErr_Occurred())
            goto done;
        if (i < 0 || i >= n) {
            PyErr_SetString(PyExc_ValueError, "ekf_rts: wrap index");
            goto done;
        }
        d[i] = wrap_pi(d[i]);
    }
    double *vxo = DATA(xo), *vPo = DATA(Po);
    for (Py_ssize_t i = 0; i < n; i++) {
        double acc = X[0 * n + i] * d[0];
        for (Py_ssize_t j = 1; j < n; j++)
            acc += X[j * n + i] * d[j];
        vxo[i] = vxa[i] + acc;
    }
    for (size_t i = 0; i < nn; i++)
        D[i] = vPs[i] - vPp[i];
    /* T = C @ D:  T[i][j] = sum_k X[k][i] D[k][j] */
    for (Py_ssize_t i = 0; i < n; i++) {
        double *t = T + i * n;
        for (Py_ssize_t j = 0; j < n; j++)
            t[j] = X[0 * n + i] * D[0 * n + j];
        for (Py_ssize_t k = 1; k < n; k++) {
            double c = X[k * n + i];
            const double *dr = D + k * n;
            for (Py_ssize_t j = 0; j < n; j++)
                t[j] += c * dr[j];
        }
    }
    /* Ps_out = Pa + T @ C.T:  (T @ C.T)[i][j] = sum_k T[i][k] X[k][j] */
    for (Py_ssize_t i = 0; i < n; i++) {
        const double *t = T + i * n;
        double *o = vPo + i * n;
        for (Py_ssize_t j = 0; j < n; j++)
            o[j] = t[0] * X[0 * n + j];
        for (Py_ssize_t k = 1; k < n; k++) {
            double c = t[k];
            const double *xr = X + k * n;
            for (Py_ssize_t j = 0; j < n; j++)
                o[j] += c * xr[j];
        }
        for (Py_ssize_t j = 0; j < n; j++)
            o[j] = vPa[i * n + j] + o[j];
    }
    res = Py_True;
    Py_INCREF(res);
done:
    PyMem_Free(work);
    PyMem_Free(piv);
    view_drop(&Pp); view_drop(&F); view_drop(&rho); view_drop(&Pa); view_drop(&xa); view_drop(&xp);
    view_drop(&xs); view_drop(&Ps); view_drop(&xo); view_drop(&Po);
    return res;
}

/* ====================================================================================================
 * interp_all(xs, ys, qs): trackers/view.py `_interp_all` for lists or tuples of floats and small ints
 * (anything else: Fallback)
 * ==================================================================================================== */
/* a plain float, or a plain int small enough to be exact as a double (Python's mixed arithmetic then gives
 * what the double arithmetic gives) */
static int get_num(PyObject *o, double *v)
{
    if (PyFloat_CheckExact(o)) {
        *v = PyFloat_AsDouble(o);
        return 1;
    }
    if (PyLong_CheckExact(o)) {
        int over = 0;
        long long k = PyLong_AsLongLongAndOverflow(o, &over);
        if (over || k > 9007199254740992LL || k < -9007199254740992LL)
            return 0;
        *v = (double)k;
        return 1;
    }
    return 0;
}

/* item i of a list or a tuple (borrowed) */
static inline PyObject *seq_item(PyObject *o, int is_list, Py_ssize_t i)
{
    return is_list ? PyList_GetItem(o, i) : PyTuple_GetItem(o, i);
}

static PyObject *n_interp_all(PyObject *self, PyObject *args)
{
    PyObject *xs, *ys, *qs;
    if (!PyArg_ParseTuple(args, "OOO", &xs, &ys, &qs))
        return NULL;
    int lx = PyList_CheckExact(xs), ly = PyList_CheckExact(ys), lq = PyList_CheckExact(qs);
    if ((!lx && !PyTuple_CheckExact(xs)) || (!ly && !PyTuple_CheckExact(ys)) || (!lq && !PyTuple_CheckExact(qs))) {
        PyErr_SetNone(Fallback);
        return NULL;
    }
    Py_ssize_t nx = lx ? PyList_Size(xs) : PyTuple_Size(xs), ny = ly ? PyList_Size(ys) : PyTuple_Size(ys);
    Py_ssize_t nq = lq ? PyList_Size(qs) : PyTuple_Size(qs);
    Py_ssize_t n = nx < ny ? nx : ny;
    if (n < 1 || n > 4096) {                        /* an empty table: the reference's own answer */
        PyErr_SetNone(Fallback);
        return NULL;
    }
    double sx[64], sy[64], *X = sx, *Y = sy;
    char sn[64], *isnone = sn;                      /* ys may hold None */
    PyObject *out = NULL;
    void *heap = NULL;
    if (n > 64) {
        heap = PyMem_Malloc((size_t)n * (2 * sizeof(double) + 1));
        if (heap == NULL)
            return PyErr_NoMemory();
        X = (double *)heap;
        Y = X + n;
        isnone = (char *)(Y + n);
    }
    for (Py_ssize_t i = 0; i < n; i++) {
        PyObject *y = seq_item(ys, ly, i);
        isnone[i] = (y == Py_None);
        Y[i] = 0.0;
        if (!get_num(seq_item(xs, lx, i), &X[i]) || (!isnone[i] && !get_num(y, &Y[i]))) {
            PyErr_SetNone(Fallback);
            goto done;
        }
    }
    out = PyList_New(nq);
    if (out == NULL)
        goto done;
    Py_ssize_t i = 1;
    int have_prev = 0;
    double prev = 0.0;
    PyObject *y0 = seq_item(ys, ly, 0);
    for (Py_ssize_t q = 0; q < nq; q++) {
        double x;
        PyObject *item;
        if (!get_num(seq_item(qs, lq, q), &x)) {
            Py_DECREF(out);
            out = NULL;
            PyErr_SetNone(Fallback);
            goto done;
        }
        if (x <= X[0]) {
            item = y0;
            Py_INCREF(item);
        } else {
            if (have_prev && !(x >= prev))
                i = 1;
            have_prev = 1;
            prev = x;
            while (i < n && !(x <= X[i]))
                i++;
            if (i >= n || isnone[i - 1] || isnone[i]) {
                item = Py_None;
                Py_INCREF(item);
            } else {
                double a = Y[i - 1], b = Y[i];
                double den = X[i] - X[i - 1];
                if (den == 0.0)                       /* Python: (xs[i] - xs[i - 1]) or 1.0 */
                    den = 1.0;
                double t = (x - X[i - 1]) / den;
                item = PyFloat_FromDouble(a + (b - a) * t);
                if (item == NULL) {
                    Py_DECREF(out);
                    out = NULL;
                    goto done;
                }
            }
        }
        PyList_SetItem(out, q, item);
    }
done:
    PyMem_Free(heap);
    return out;
}

/* ====================================================================================================
 * The lane tracker's array work (trackers/lanes.py is the reference; names follow it). Elementwise
 * steps as numpy does them; `mean` over the gate points adds in numpy's order (seq_sum below).
 * ==================================================================================================== */
#define N_LANE 4
#define N_LINES 6

/* The mean over the gate points, as numpy takes it in trackers/lanes.py: `a[:, gate].mean(axis=1)`. The
 * selection a[:, gate] comes out in column order, and numpy then adds each line's values one after the
 * other, in order (not pairwise). tests/test_native.py holds this against numpy; a numpy that adds otherwise
 * fails that test, and the tracker's two fit measures then agree with the reference to rounding. */
static double seq_sum(const double *a, Py_ssize_t n)
{
    double res = a[0];
    for (Py_ssize_t i = 1; i < n; i++)
        res += a[i];
    return res;
}

static inline double clamp01(double v)
{
    double m = (v > 0.0) ? v : 0.0;              /* Python: min(1.0, max(0.0, v)) */
    return (m < 1.0) ? m : 1.0;
}

static void sort_small(double *a, Py_ssize_t n)
{
    for (Py_ssize_t i = 1; i < n; i++) {
        double v = a[i];
        Py_ssize_t j = i - 1;
        while (j >= 0 && a[j] > v) {
            a[j + 1] = a[j];
            j--;
        }
        a[j + 1] = v;
    }
}

/* a list of n plain floats or ints into `out`; 0 = not such a list (the reference decides), else 1 and
 * *finite = every value is finite */
static int list_row(PyObject *lst, Py_ssize_t n, double *out, int *finite)
{
    if (lst == NULL || !PyList_CheckExact(lst) || PyList_Size(lst) != n)
        return 0;
    int fin = 1;
    for (Py_ssize_t i = 0; i < n; i++) {
        PyObject *v = PyList_GetItem(lst, i);
        double d;
        if (PyFloat_CheckExact(v))
            d = PyFloat_AsDouble(v);
        else if (PyLong_CheckExact(v)) {
            d = PyLong_AsDouble(v);
            if (d == -1.0 && PyErr_Occurred()) {
                PyErr_Clear();
                return 0;
            }
        } else
            return 0;
        if (!isfinite(d))
            fin = 0;
        out[i] = d;
    }
    *finite = fin;
    return 1;
}

/* lanes_measure(srcs, x, std_x0, std_x1, std_bad, std_good, y, s, prob, ok)
 * RoadGeometryTracker._measure after the distances are known, for the usual frame: six dicts whose "y" and
 * "y_std" are lists of len(x) numbers. Fills y[6,n], s[6,n], prob[6], ok[6]. Anything else: Fallback. */
static PyObject *n_lanes_measure(PyObject *self, PyObject *args)
{
    PyObject *srcs, *ox, *oy, *os, *oprob, *ook;
    double x0, x1, bad, good;
    View x, y, sv, prob, ok;
    x.held = y.held = sv.held = prob.held = ok.held = 0;
    PyObject *res = NULL;
    double *rows = NULL;
    if (!PyArg_ParseTuple(args, "OOddddOOOO", &srcs, &ox, &x0, &x1, &bad, &good, &oy, &os, &oprob, &ook))
        return NULL;
    if (!PyList_CheckExact(srcs) || PyList_Size(srcs) != N_LINES) {
        PyErr_SetNone(Fallback);
        return NULL;
    }
    if (view_get(ox, &x, 0, 'd', 1, "x") < 0 || view_get(oy, &y, 1, 'd', 2, "y") < 0 ||
        view_get(os, &sv, 1, 'd', 2, "s") < 0 || view_get(oprob, &prob, 1, 'd', 1, "prob") < 0 ||
        view_get(ook, &ok, 1, 'b', 1, "ok") < 0)
        goto done;
    Py_ssize_t n = x.b.shape[0];
    if (n < 1 || n > 1024 || y.b.shape[0] != N_LINES || y.b.shape[1] != n || sv.b.shape[0] != N_LINES ||
        sv.b.shape[1] != n || prob.b.shape[0] != N_LINES || ok.b.shape[0] != N_LINES) {
        PyErr_SetString(PyExc_ValueError, "lanes_measure: shapes");
        goto done;
    }
    rows = (double *)PyMem_Malloc((size_t)(2 * N_LINES + 1) * (size_t)n * sizeof(double));
    if (rows == NULL) {
        PyErr_NoMemory();
        goto done;
    }
    int fin[2 * N_LINES];
    double pv[N_LANE];
    PyObject *ky = PyUnicode_FromString("y"), *ks = PyUnicode_FromString("y_std"), *kp = PyUnicode_FromString("prob");
    int good_shape = (ky && ks && kp);
    for (int i = 0; good_shape && i < N_LINES; i++) {
        PyObject *d = PyList_GetItem(srcs, i);
        if (d == NULL || !PyDict_CheckExact(d)) {
            good_shape = 0;
            break;
        }
        PyObject *ly = PyDict_GetItemWithError(d, ky), *ls = PyDict_GetItemWithError(d, ks);
        if (!list_row(ly, n, rows + (Py_ssize_t)i * n, &fin[i]) ||
            !list_row(ls, n, rows + (Py_ssize_t)(N_LINES + i) * n, &fin[N_LINES + i])) {
            good_shape = 0;
            break;
        }
        if (i < N_LANE) {
            /* prob: min(1, max(0, float(p))) if p is a finite int / float (a bool is an int), else 0 */
            PyObject *p = PyDict_GetItemWithError(d, kp);
            double v = 0.0;
            if (p == NULL || p == Py_None)
                v = 0.0;
            else if (PyFloat_CheckExact(p)) {
                v = PyFloat_AsDouble(p);
                v = isfinite(v) ? clamp01(v) : 0.0;
            } else if (PyLong_CheckExact(p) || p == Py_True || p == Py_False) {
                v = PyLong_AsDouble(p);
                if (v == -1.0 && PyErr_Occurred()) {
                    PyErr_Clear();
                    good_shape = 0;
                    break;
                }
                v = clamp01(v);
            } else {
                good_shape = 0;                     /* any other type: the reference decides */
                break;
            }
            pv[i] = v;
        }
    }
    Py_XDECREF(ky); Py_XDECREF(ks); Py_XDECREF(kp);
    if (PyErr_Occurred())
        goto done;
    if (!good_shape) {
        PyErr_SetNone(Fallback);
        goto done;
    }
    const double *xv = DATA(x);
    double *Y = DATA(y), *S = DATA(sv), *PR = DATA(prob);
    char *OK = (char *)ok.b.buf;
    double *tmp = rows + (Py_ssize_t)(2 * N_LINES) * n;
    Py_ssize_t n_near = 0;
    for (Py_ssize_t j = 0; j < n; j++)
        if (xv[j] >= x0 && xv[j] <= x1)
            n_near++;
    for (int i = 0; i < N_LINES; i++) {
        double *yo = Y + (Py_ssize_t)i * n, *so = S + (Py_ssize_t)i * n;
        if (!fin[i]) {                               /* offsets not finite: the line is not there this frame */
            for (Py_ssize_t j = 0; j < n; j++) {
                yo[j] = 0.0;
                so[j] = 1.0;
            }
            PR[i] = 0.0;
            OK[i] = 0;
            continue;
        }
        const double *yi = rows + (Py_ssize_t)i * n, *si = rows + (Py_ssize_t)(N_LINES + i) * n;
        double dflt = (i < N_LANE) ? 0.3 : 1.0;
        int have_s = fin[N_LINES + i];
        for (Py_ssize_t j = 0; j < n; j++) {
            double sj = have_s ? si[j] : dflt;
            yo[j] = yi[j];
            so[j] = (sj >= 1e-3) ? sj : 1e-3;        /* np.maximum(si, 1e-3) */
        }
        if (i < N_LANE)
            PR[i] = pv[i];
        else {
            /* the median of the near-range deviations (all of them when no point is near) */
            Py_ssize_t m = 0;
            for (Py_ssize_t j = 0; j < n; j++)
                if (n_near == 0 || (xv[j] >= x0 && xv[j] <= x1))
                    tmp[m++] = have_s ? si[j] : dflt;
            sort_small(tmp, m);
            Py_ssize_t h = m / 2;
            double sd = (m % 2) ? tmp[h] : (tmp[h - 1] + tmp[h]) / 2.0;
            PR[i] = clamp01((bad - sd) / (bad - good));
        }
        OK[i] = 1;
    }
    res = Py_None;
    Py_INCREF(res);
done:
    PyMem_Free(rows);
    view_drop(&x); view_drop(&y); view_drop(&sv); view_drop(&prob); view_drop(&ok);
    return res;
}

/* lanes_predict(p, x, q0, q1, dt, cap):  p = minimum(p + (q0 + q1 * x) ** 2 * dt, cap)   in place */
static PyObject *n_lanes_predict(PyObject *self, PyObject *args)
{
    PyObject *op, *ox;
    double q0, q1, dt, cap;
    View p, x;
    p.held = x.held = 0;
    PyObject *res = NULL;
    if (!PyArg_ParseTuple(args, "OOdddd", &op, &ox, &q0, &q1, &dt, &cap))
        return NULL;
    if (view_get(op, &p, 1, 'd', 2, "p") < 0 || view_get(ox, &x, 0, 'd', 1, "x") < 0)
        goto done;
    Py_ssize_t n = x.b.shape[0], m = p.b.shape[0];
    if (p.b.shape[1] != n) {
        PyErr_SetString(PyExc_ValueError, "lanes_predict: shapes");
        goto done;
    }
    const double *xv = DATA(x);
    double *P = DATA(p);
    for (Py_ssize_t j = 0; j < n; j++) {
        double b = q0 + q1 * xv[j];
        double q = b * b * dt;
        for (Py_ssize_t i = 0; i < m; i++) {
            double v = P[i * n + j] + q;
            P[i * n + j] = (v <= cap || v != v) ? v : cap;      /* np.minimum (a NaN stays) */
        }
    }
    res = Py_None;
    Py_INCREF(res);
done:
    view_drop(&p); view_drop(&x);
    return res;
}

/* lanes_fit(x, zy, zs, prob, y, p, par, r, nu, gate, jumps, nsigs)
 * par = [r_floor, r_scale, edge_r_floor, edge_r_scale, gate_x0, gate_x1, on_conf, low_prob_inflate]
 *   r     = maximum(floor, scale * zs) per line kind, times low_prob_inflate where prob < on_conf
 *   gate  = (x >= gate_x0) & (x <= gate_x1), all True when none is
 *   nu    = zy - y
 *   jumps = mean over the gate points of |nu|;  nsigs = mean of |nu| / sqrt(p + r^2) */
static PyObject *n_lanes_fit(PyObject *self, PyObject *args)
{
    PyObject *ox, *ozy, *ozs, *oprob, *oy, *op, *opar, *orr, *onu, *og, *oj, *ons;
    View x, zy, zs, prob, y, p, par, r, nu, g, jm, ns;
    x.held = zy.held = zs.held = prob.held = y.held = p.held = par.held = r.held = nu.held = g.held = jm.held = ns.held = 0;
    PyObject *res = NULL;
    double *tmp = NULL;
    if (!PyArg_ParseTuple(args, "OOOOOOOOOOOO", &ox, &ozy, &ozs, &oprob, &oy, &op, &opar, &orr, &onu, &og, &oj, &ons))
        return NULL;
    if (view_get(ox, &x, 0, 'd', 1, "x") < 0 || view_get(ozy, &zy, 0, 'd', 2, "zy") < 0 ||
        view_get(ozs, &zs, 0, 'd', 2, "zs") < 0 || view_get(oprob, &prob, 0, 'd', 1, "prob") < 0 ||
        view_get(oy, &y, 0, 'd', 2, "y") < 0 || view_get(op, &p, 0, 'd', 2, "p") < 0 ||
        view_get(opar, &par, 0, 'd', 1, "par") < 0 || view_get(orr, &r, 1, 'd', 2, "r") < 0 ||
        view_get(onu, &nu, 1, 'd', 2, "nu") < 0 || view_get(og, &g, 1, 'b', 1, "gate") < 0 ||
        view_get(oj, &jm, 1, 'd', 1, "jumps") < 0 || view_get(ons, &ns, 1, 'd', 1, "nsigs") < 0)
        goto done;
    Py_ssize_t n = x.b.shape[0];
#define SHAPE2(v) ((v).b.shape[0] == N_LINES && (v).b.shape[1] == n)
    if (n < 1 || !SHAPE2(zy) || !SHAPE2(zs) || !SHAPE2(y) || !SHAPE2(p) || !SHAPE2(r) || !SHAPE2(nu) ||
        prob.b.shape[0] != N_LINES || par.b.shape[0] != 8 || g.b.shape[0] != n || jm.b.shape[0] != N_LINES ||
        ns.b.shape[0] != N_LINES) {
        PyErr_SetString(PyExc_ValueError, "lanes_fit: shapes");
        goto done;
    }
#undef SHAPE2
    tmp = (double *)PyMem_Malloc((size_t)(2 * n) * sizeof(double));
    if (tmp == NULL) {
        PyErr_NoMemory();
        goto done;
    }
    const double *c = DATA(par), *xv = DATA(x), *ZY = DATA(zy), *ZS = DATA(zs), *PR = DATA(prob), *Y = DATA(y), *P = DATA(p);
    double *R = DATA(r), *NU = DATA(nu), *J = DATA(jm), *NS = DATA(ns);
    char *G = (char *)g.b.buf;
    Py_ssize_t ng = 0;
    for (Py_ssize_t j = 0; j < n; j++) {
        G[j] = (xv[j] >= c[4] && xv[j] <= c[5]);
        ng += G[j];
    }
    if (ng == 0) {
        for (Py_ssize_t j = 0; j < n; j++)
            G[j] = 1;
        ng = n;
    }
    for (int i = 0; i < N_LINES; i++) {
        double floor_ = (i < N_LANE) ? c[0] : c[2], scale = (i < N_LANE) ? c[1] : c[3];
        double infl = (PR[i] < c[6]) ? c[7] : 1.0;
        Py_ssize_t m = 0;
        for (Py_ssize_t j = 0; j < n; j++) {
            double v = scale * ZS[i * n + j];
            double rr = (floor_ >= v) ? floor_ : v;             /* np.maximum(floor, scale * zs) */
            rr = rr * infl;
            R[i * n + j] = rr;
            double d = ZY[i * n + j] - Y[i * n + j];
            NU[i * n + j] = d;
            if (G[j]) {
                double an = fabs(d);
                tmp[m] = an;
                tmp[n + m] = an / sqrt(P[i * n + j] + rr * rr);
                m++;
            }
        }
        J[i] = seq_sum(tmp, ng) / (double)ng;
        NS[i] = seq_sum(tmp + n, ng) / (double)ng;
    }
    res = Py_None;
    Py_INCREF(res);
done:
    PyMem_Free(tmp);
    view_drop(&x); view_drop(&zy); view_drop(&zs); view_drop(&prob); view_drop(&y); view_drop(&p); view_drop(&par);
    view_drop(&r); view_drop(&nu); view_drop(&g); view_drop(&jm); view_drop(&ns);
    return res;
}

/* lanes_apply(y, p, r, nu, upd): for every line i in `upd` (a list of ints)
 *   k = p / (p + r * r);  y += k * nu;  p = p * (1.0 - k) */
static PyObject *n_lanes_apply(PyObject *self, PyObject *args)
{
    PyObject *oy, *op, *orr, *onu, *upd;
    View y, p, r, nu;
    y.held = p.held = r.held = nu.held = 0;
    PyObject *res = NULL;
    if (!PyArg_ParseTuple(args, "OOOOO", &oy, &op, &orr, &onu, &upd))
        return NULL;
    if (!PyList_CheckExact(upd)) {
        PyErr_SetString(PyExc_TypeError, "lanes_apply: upd is a list");
        return NULL;
    }
    if (view_get(oy, &y, 1, 'd', 2, "y") < 0 || view_get(op, &p, 1, 'd', 2, "p") < 0 ||
        view_get(orr, &r, 0, 'd', 2, "r") < 0 || view_get(onu, &nu, 0, 'd', 2, "nu") < 0)
        goto done;
    Py_ssize_t m = y.b.shape[0], n = y.b.shape[1];
    if (p.b.shape[0] != m || p.b.shape[1] != n || r.b.shape[0] != m || r.b.shape[1] != n || nu.b.shape[0] != m ||
        nu.b.shape[1] != n) {
        PyErr_SetString(PyExc_ValueError, "lanes_apply: shapes");
        goto done;
    }
    Py_ssize_t nu_ = PyList_Size(upd);
    for (Py_ssize_t q = 0; q < nu_; q++) {
        Py_ssize_t i = PyLong_AsSsize_t(PyList_GetItem(upd, q));
        if (i == -1 && PyErr_Occurred())
            goto done;
        if (i < 0 || i >= m) {
            PyErr_SetString(PyExc_ValueError, "lanes_apply: line index");
            goto done;
        }
    }
    double *Y = DATA(y), *P = DATA(p);
    const double *R = DATA(r), *NU = DATA(nu);
    for (Py_ssize_t q = 0; q < nu_; q++) {
        Py_ssize_t i = PyLong_AsSsize_t(PyList_GetItem(upd, q));
        for (Py_ssize_t j = 0; j < n; j++) {
            double pu = P[i * n + j], ru = R[i * n + j];
            double k = pu / (pu + ru * ru);
            Y[i * n + j] += k * NU[i * n + j];
            P[i * n + j] = pu * (1.0 - k);
        }
    }
    res = Py_None;
    Py_INCREF(res);
done:
    view_drop(&y); view_drop(&p); view_drop(&r); view_drop(&nu);
    return res;
}

/* round_lists(a, decimals, root): np.round(a, decimals).tolist() of a float array of one or two dimensions
 * (root true: of np.sqrt(a)); numpy rounds as rint(v * 10^decimals) / 10^decimals */
static PyObject *n_round_lists(PyObject *self, PyObject *args)
{
    PyObject *oa;
    int dec, root;
    View a;
    a.held = 0;
    if (!PyArg_ParseTuple(args, "Oip", &oa, &dec, &root))
        return NULL;
    if (dec < 1 || dec > 22) {
        PyErr_SetString(PyExc_ValueError, "round_lists: decimals 1..22");
        return NULL;
    }
    if (PyObject_GetBuffer(oa, &a.b, PyBUF_C_CONTIGUOUS | PyBUF_FORMAT) < 0)
        return NULL;
    a.held = 1;
    const char *f = a.b.format ? a.b.format : "B";
    if (*f == '@' || *f == '=' || *f == '<')
        f++;
    if (!(f[0] == 'd' && f[1] == 0) || a.b.itemsize != 8 || a.b.ndim < 1 || a.b.ndim > 2) {
        view_drop(&a);
        PyErr_SetString(PyExc_TypeError, "round_lists: a float64 array of one or two dimensions");
        return NULL;
    }
    Py_ssize_t rows = a.b.ndim == 2 ? a.b.shape[0] : 1, n = a.b.ndim == 2 ? a.b.shape[1] : a.b.shape[0];
    const double *v = DATA(a);
    double sc = POW10[dec];
    PyObject *out = a.b.ndim == 2 ? PyList_New(rows) : NULL;
    if (a.b.ndim == 2 && out == NULL) {
        view_drop(&a);
        return NULL;
    }
    for (Py_ssize_t i = 0; i < rows; i++) {
        PyObject *row = PyList_New(n);
        if (row == NULL)
            goto fail;
        for (Py_ssize_t j = 0; j < n; j++) {
            double d = v[i * n + j];
            if (root)
                d = sqrt(d);
            PyObject *o = PyFloat_FromDouble(rint(d * sc) / sc);
            if (o == NULL) {
                Py_DECREF(row);
                goto fail;
            }
            PyList_SetItem(row, j, o);
        }
        if (a.b.ndim == 2)
            PyList_SetItem(out, i, row);
        else
            out = row;
    }
    view_drop(&a);
    return out;
fail:
    Py_XDECREF(out);
    view_drop(&a);
    return NULL;
}

/* ==================================================================================================== */
static PyMethodDef methods[] = {
    {"dumps", n_dumps, METH_O, "json.dumps(obj, separators=(',', ':')); raises Fallback where json must decide"},
    {"loads", n_loads, METH_O, "json.loads(bytes | str); raises Fallback where json must decide"},
    {"set_rounded_type", n_set_rounded_type, METH_O, "the tuple subclass (values, digits) dumps writes rounded"},
    {"round", n_round, METH_VARARGS, "builtins.round"},
    {"round_list", n_round_list, METH_VARARGS, "[v if v is None else round(v, n) for v in values]"},
    {"interp_all", n_interp_all, METH_VARARGS, "trackers.view._interp_all for lists of floats"},
    {"ekf_predict", n_ekf_predict, METH_VARARGS, "EgoModel.predict into given arrays"},
    {"ekf_project", n_ekf_project, METH_VARARGS, "ph = P[:, idx] @ coef into ph; returns coef @ ph[idx]"},
    {"ekf_update", n_ekf_update, METH_VARARGS, "one scalar row applied to x and P in place"},
    {"ekf_sym", n_ekf_sym, METH_O, "P = 0.5 * (P + P.T) in place"},
    {"ekf_rts", n_ekf_rts, METH_VARARGS, "one smoother step back over a prediction"},
    {"lanes_measure", n_lanes_measure, METH_VARARGS, "the lane tracker's measurement arrays from a road message's lines"},
    {"lanes_predict", n_lanes_predict, METH_VARARGS, "the lane tracker's process noise, in place"},
    {"lanes_fit", n_lanes_fit, METH_VARARGS, "measurement noise, gate, innovation and fit of every line"},
    {"lanes_apply", n_lanes_apply, METH_VARARGS, "the scalar-gain update of the fitted lines, in place"},
    {"round_lists", n_round_lists, METH_VARARGS, "np.round(a, decimals).tolist(), optionally of np.sqrt(a)"},
    {NULL, NULL, 0, NULL}};

static struct PyModuleDef moduledef = {PyModuleDef_HEAD_INIT, "_chris_native",
                                       "chris-integrator's native core (see chris_integrator/native)", -1, methods,
                                       NULL, NULL, NULL, NULL};

PyMODINIT_FUNC PyInit__chris_native(void)
{
    PyObject *m = PyModule_Create(&moduledef);
    if (m == NULL)
        return NULL;
    Fallback = PyErr_NewException("chris_integrator.native.Fallback", NULL, NULL);
    PyObject *bi = PyImport_ImportModule("builtins");
    if (Fallback == NULL || bi == NULL)
        goto fail;
    builtin_round = PyObject_GetAttrString(bi, "round");
    Py_DECREF(bi);
    if (builtin_round == NULL)
        goto fail;
    Py_INCREF(Fallback);
    if (PyModule_AddObject(m, "Fallback", Fallback) < 0 || PyModule_AddIntConstant(m, "VERSION", NATIVE_VERSION) < 0)
        goto fail;
#ifdef __FP_FAST_FMA
    /* informational: the target has a fused multiply-add; the build must not contract (see the Makefile) */
    PyModule_AddIntConstant(m, "TARGET_HAS_FMA", 1);
#else
    PyModule_AddIntConstant(m, "TARGET_HAS_FMA", 0);
#endif
    return m;
fail:
    Py_DECREF(m);
    return NULL;
}
