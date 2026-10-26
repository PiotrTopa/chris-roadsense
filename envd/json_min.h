// Minimal JSON reader (header-only) for envd's class file (models/envd-classes.json): objects,
// arrays, strings (ASCII + \uXXXX as UTF-8), numbers, true/false/null. No streaming, no comments.
#pragma once

#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace roadsense {
namespace envd {

struct Json {
    enum Type { Null, Bool, Number, String, Array, Object };
    Type type = Null;
    bool b = false;
    double num = 0;
    std::string str;
    std::vector<Json> arr;
    std::vector<std::pair<std::string, Json>> obj;

    const Json *get(const std::string &key) const {
        if (type != Object) return nullptr;
        for (const auto &kv : obj)
            if (kv.first == key) return &kv.second;
        return nullptr;
    }
    size_t size() const { return type == Array ? arr.size() : (type == Object ? obj.size() : 0); }
    const Json &operator[](size_t i) const { return arr[i]; }
};

class JsonParser {
public:
    static bool parse(const std::string &s, Json *out, std::string *err) {
        JsonParser p(s);
        p.ws();
        if (!p.value(out)) {
            if (err) *err = p.err_ + " at byte " + std::to_string(p.i_);
            return false;
        }
        p.ws();
        if (p.i_ != s.size()) {
            if (err) *err = "trailing data at byte " + std::to_string(p.i_);
            return false;
        }
        return true;
    }

private:
    explicit JsonParser(const std::string &s) : s_(s) {}
    const std::string &s_;
    size_t i_ = 0;
    std::string err_;

    bool fail(const char *m) {
        if (err_.empty()) err_ = m;
        return false;
    }
    void ws() {
        while (i_ < s_.size() && (s_[i_] == ' ' || s_[i_] == '\n' || s_[i_] == '\r' || s_[i_] == '\t')) ++i_;
    }
    bool lit(const char *w, Json *o, Json::Type t, bool b) {
        const size_t n = strlen(w);
        if (s_.compare(i_, n, w) != 0) return fail("bad literal");
        i_ += n;
        o->type = t;
        o->b = b;
        return true;
    }
    bool value(Json *o) {
        if (i_ >= s_.size()) return fail("unexpected end");
        const char c = s_[i_];
        if (c == '{') return object(o);
        if (c == '[') return array(o);
        if (c == '"') {
            o->type = Json::String;
            return string(&o->str);
        }
        if (c == 't') return lit("true", o, Json::Bool, true);
        if (c == 'f') return lit("false", o, Json::Bool, false);
        if (c == 'n') return lit("null", o, Json::Null, false);
        return number(o);
    }
    bool number(Json *o) {
        const char *start = s_.c_str() + i_;
        char *end = nullptr;
        const double v = strtod(start, &end);
        if (end == start) return fail("bad value");
        i_ += size_t(end - start);
        o->type = Json::Number;
        o->num = v;
        return true;
    }
    static void put_utf8(std::string *out, unsigned cp) {
        if (cp < 0x80) out->push_back(char(cp));
        else if (cp < 0x800) {
            out->push_back(char(0xC0 | (cp >> 6)));
            out->push_back(char(0x80 | (cp & 0x3F)));
        } else {
            out->push_back(char(0xE0 | (cp >> 12)));
            out->push_back(char(0x80 | ((cp >> 6) & 0x3F)));
            out->push_back(char(0x80 | (cp & 0x3F)));
        }
    }
    bool string(std::string *out) {
        ++i_;  // opening quote
        out->clear();
        while (i_ < s_.size()) {
            const char c = s_[i_++];
            if (c == '"') return true;
            if (c != '\\') {
                out->push_back(c);
                continue;
            }
            if (i_ >= s_.size()) break;
            const char e = s_[i_++];
            switch (e) {
                case '"': out->push_back('"'); break;
                case '\\': out->push_back('\\'); break;
                case '/': out->push_back('/'); break;
                case 'b': out->push_back('\b'); break;
                case 'f': out->push_back('\f'); break;
                case 'n': out->push_back('\n'); break;
                case 'r': out->push_back('\r'); break;
                case 't': out->push_back('\t'); break;
                case 'u': {
                    if (i_ + 4 > s_.size()) return fail("bad \\u escape");
                    const unsigned cp = unsigned(strtoul(s_.substr(i_, 4).c_str(), nullptr, 16));
                    i_ += 4;
                    put_utf8(out, cp);
                    break;
                }
                default: return fail("bad escape");
            }
        }
        return fail("unterminated string");
    }
    bool array(Json *o) {
        ++i_;
        o->type = Json::Array;
        ws();
        if (i_ < s_.size() && s_[i_] == ']') {
            ++i_;
            return true;
        }
        for (;;) {
            ws();
            o->arr.emplace_back();
            if (!value(&o->arr.back())) return false;
            ws();
            if (i_ >= s_.size()) return fail("unterminated array");
            if (s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (s_[i_] == ']') {
                ++i_;
                return true;
            }
            return fail("expected , or ]");
        }
    }
    bool object(Json *o) {
        ++i_;
        o->type = Json::Object;
        ws();
        if (i_ < s_.size() && s_[i_] == '}') {
            ++i_;
            return true;
        }
        for (;;) {
            ws();
            if (i_ >= s_.size() || s_[i_] != '"') return fail("expected key");
            std::string key;
            if (!string(&key)) return false;
            ws();
            if (i_ >= s_.size() || s_[i_] != ':') return fail("expected :");
            ++i_;
            ws();
            o->obj.emplace_back(std::move(key), Json());
            if (!value(&o->obj.back().second)) return false;
            ws();
            if (i_ >= s_.size()) return fail("unterminated object");
            if (s_[i_] == ',') {
                ++i_;
                continue;
            }
            if (s_[i_] == '}') {
                ++i_;
                return true;
            }
            return fail("expected , or }");
        }
    }
};

}  // namespace envd
}  // namespace roadsense
