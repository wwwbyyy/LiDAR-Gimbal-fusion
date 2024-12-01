#pragma once

#include <yaml-cpp/yaml.h>

#include "core.h"

// clang-format off

namespace reflcpp::yaml {
    template<typename T>
    std::enable_if_t<is_reflectable_v<T>, YAML::Node> encode(const T &obj) {
        YAML::Node node;
        fields_foreach_recursive<T>([&](auto field) {
            node[field.name().data()] = field.get(obj);
        });
        return node;
    }

    template<typename T>
    std::enable_if_t<is_reflectable_v<T>, bool> decode(const YAML::Node& node, T& obj) {
        try {
            fields_foreach_recursive<T>([&](auto field) {
                auto fieldName = field.name().data();
                if (node[fieldName]) {
                    field.set(obj, node[fieldName].template as<typename decltype(field)::type>());
                } else {
                    std::cerr << "Field " << fieldName << " not found in YAML node." << std::endl;
                }
            });
            return true;
        } catch (const YAML::Exception& e) {
            std::cerr << "YAML Exception: " << e.what() << std::endl;
            return false;
        }
    }
}

#ifdef REFLCPP_YAML
#undef REFLCPP_YAML
#endif
#define REFLCPP_YAML(Class, ...)                                                \
    template<> struct YAML::convert<Class> {                                    \
        static Node encode(const Class &obj) {                                  \
            return reflcpp::yaml::encode(obj);                                  \
        }                                                                       \
        static bool decode(const Node& node, Class& obj) {                      \
            return reflcpp::yaml::decode(node, obj);                            \
        }                                                                       \
    }; 
