#include "classes.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "json_min.h"

namespace roadsense {
namespace envd {

namespace {

const Json &need(const Json &o, const char *key, Json::Type t, const std::string &ctx) {
    const Json *v = o.get(key);
    if (!v || v->type != t) throw std::runtime_error(ctx + ": missing or wrong-typed \"" + key + "\"");
    return *v;
}

}  // namespace

ClassFile load_class_file(const std::string &path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("cannot open " + path);
    std::stringstream ss;
    ss << f.rdbuf();
    Json root;
    std::string err;
    if (!JsonParser::parse(ss.str(), &root, &err)) throw std::runtime_error(path + ": " + err);
    if (root.type != Json::Object) throw std::runtime_error(path + ": not an object");
    ClassFile cf;
    cf.model = need(root, "model", Json::String, path).str;
    if (const Json *p = root.get("pretrained")) cf.pretrained = p->str;
    cf.embed_dim = int(need(root, "embed_dim", Json::Number, path).num);
    cf.image_size = int(need(root, "image_size", Json::Number, path).num);
    cf.logit_scale = float(need(root, "logit_scale", Json::Number, path).num);
    const Json &mean = need(root, "mean", Json::Array, path), &std = need(root, "std", Json::Array, path);
    if (mean.size() != 3 || std.size() != 3) throw std::runtime_error(path + ": mean/std must have 3 values");
    for (int i = 0; i < 3; ++i) {
        cf.mean[i] = float(mean[i].num);
        cf.std[i] = float(std[i].num);
    }
    const Json &groups = need(root, "groups", Json::Object, path);
    for (const auto &kv : groups.obj) {
        ClassGroup g;
        g.name = kv.first;
        const std::string ctx = path + " group " + g.name;
        const Json &classes = need(kv.second, "classes", Json::Array, ctx);
        const Json &emb = need(kv.second, "emb", Json::Array, ctx);
        if (classes.size() == 0 || classes.size() != emb.size())
            throw std::runtime_error(path + ": group " + g.name + ": classes/emb size mismatch");
        for (size_t i = 0; i < classes.size(); ++i) {
            if (classes[i].type != Json::String) throw std::runtime_error(path + ": class names must be strings");
            g.classes.push_back(classes[i].str);
            const Json &row = emb[i];
            if (row.type != Json::Array || int(row.size()) != cf.embed_dim)
                throw std::runtime_error(path + ": group " + g.name + " row " + std::to_string(i) + " is not " +
                                         std::to_string(cf.embed_dim) + " numbers");
            for (size_t j = 0; j < row.size(); ++j) g.emb.push_back(float(row[j].num));
        }
        cf.groups.push_back(std::move(g));
    }
    if (cf.groups.empty()) throw std::runtime_error(path + ": no groups");
    return cf;
}

}  // namespace envd
}  // namespace roadsense
