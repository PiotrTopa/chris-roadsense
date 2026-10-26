// The class file envd ships (models/envd-classes.json, written by tools/envd/text_embeddings.py):
// one text embedding per class in each group (place, weather, light), plus the image encoder's
// input size, normalisation and logit scale. The phone never runs a text encoder.
#pragma once

#include <string>
#include <vector>

namespace roadsense {
namespace envd {

struct ClassGroup {
    std::string name;                  // "place", "weather", "light"
    std::vector<std::string> classes;  // class names in embedding row order
    std::vector<float> emb;            // classes.size() x dim, L2-normalised rows
};

struct ClassFile {
    std::string model, pretrained;
    int embed_dim = 0, image_size = 0;
    float mean[3] = {0, 0, 0}, std[3] = {1, 1, 1};
    float logit_scale = 100.f;
    std::vector<ClassGroup> groups;
};

// Throws std::runtime_error on a missing or malformed file.
ClassFile load_class_file(const std::string &path);

}  // namespace envd
}  // namespace roadsense
