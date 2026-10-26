#include "output.h"

#include <cstdio>

namespace roadsense {
namespace envd {

namespace {

void num(std::string &s, double v, int digits = 3) {
    char b[32];
    snprintf(b, sizeof b, "%.*f", digits, v);
    s += b;
}

void scores(std::string &s, const ClassGroup &g, const std::vector<double> &v) {
    s += '{';
    for (size_t i = 0; i < g.classes.size(); ++i) {
        if (i) s += ',';
        s += '"' + g.classes[i] + "\":";
        num(s, v[i]);
    }
    s += '}';
}

}  // namespace

std::string env_json(const FrameMeta &m, const EnvResult &r, const std::string &model, bool with_frame_scores) {
    std::string s;
    s.reserve(2048);
    s += "{\"type\":\"env\",\"seq\":" + std::to_string(m.seq) + ",\"ts_mono_ns\":" + std::to_string(m.ts_mono_ns) +
         ",\"ts_real_ns\":" + std::to_string(m.ts_real_ns) + ",\"env_ms\":";
    num(s, r.total_ms, 1);
    s += ",\"encode_ms\":";
    num(s, r.encode_ms, 1);
    for (const GroupResult &g : r.groups) {
        s += ",\"" + g.group->name + "\":{\"label\":\"" + (g.label >= 0 ? g.group->classes[size_t(g.label)] : "") +
             "\",\"confidence\":";
        num(s, g.confidence);
        s += ",\"stable_s\":";
        num(s, g.stable_s, 1);
        s += ",\"scores\":";
        scores(s, *g.group, g.scores);
        s += ",\"frame_label\":\"" + (g.frame_label >= 0 ? g.group->classes[size_t(g.frame_label)] : "") + "\"";
        if (with_frame_scores) {
            s += ",\"frame_scores\":";
            std::vector<double> f(g.frame.begin(), g.frame.end());
            scores(s, *g.group, f);
        }
        s += '}';
    }
    s += ",\"stats\":{\"luma\":";
    num(s, r.stats.luma, 1);
    s += ",\"luma_top\":";
    num(s, r.stats.luma_top, 1);
    s += ",\"luma_bottom\":";
    num(s, r.stats.luma_bottom, 1);
    s += ",\"dark_frac\":";
    num(s, r.stats.dark_frac);
    s += ",\"bright_frac\":";
    num(s, r.stats.bright_frac);
    s += ",\"chroma\":";
    num(s, r.stats.chroma, 1);
    s += "},\"model\":\"" + model + "\"}";
    return s;
}

}  // namespace envd
}  // namespace roadsense
