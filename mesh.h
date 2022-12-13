#ifndef MODEL_H
#define MODEL_H

#include <vector>
#include <list>
#include <sstream>
#include <fstream>
#include <map>
#include "shape.h"

struct Mat4f {
    float m[4][4];
};

enum ply_type {
    PLY_TYPE_INT,
    PLY_TYPE_UINT,
    PLY_TYPE_FLOAT,
    PLY_TYPE_CHAR,
    PLY_TYPE_UCHAR  
};

static size_t ply_get_type_size(const ply_type t){
    switch (t) {
        default:return 0;
        case PLY_TYPE_FLOAT: case PLY_TYPE_INT: case PLY_TYPE_UINT: return 4;
        case PLY_TYPE_CHAR : case PLY_TYPE_UCHAR : return 1;
    }
}

static  ply_type ply_get_token_type(std::string tok) {
    if (tok == "int") {
        return PLY_TYPE_INT;
    } else if (tok == "uint") {
        return PLY_TYPE_UINT;
    } else if (tok == "float") {
        return PLY_TYPE_FLOAT;
    } else if (tok == "char") {
        return PLY_TYPE_CHAR;
    } else if (tok == "uchar") {
        return PLY_TYPE_UCHAR;
    }
    return PLY_TYPE_INT;
}

struct ply_property_storage {
    std::vector<uint8_t> data;

    void prealloc(size_t size) {
        data.reserve(size);
    }

    template<typename T> void add_val(T value) {
        uint8_t *alias_val = reinterpret_cast<uint8_t*>(&value);
        for (int i = 0; i < sizeof(T); ++i) data.push_back(alias_val[i]);
    }
};

struct ply_property {
    std::string name;
    bool is_list;
    ply_type count_type;
    ply_type type;
    ply_property_storage storage;
};

enum ply_format {
    PLY_ASCII,
    PLY_BINARY_LITTLE_ENDIAN,
    PLY_BINARY_BIG_ENDIAN,
};

#define PLY_ELEMENT_SIZE_UNKNOWN (size_t)(SIZE_MAX)

struct ply_element {
    std::string name;
    size_t count;
    std::vector<ply_property> props;

    ply_property *find_property(const char *name) {
        for (auto &p : props) {
            if (p.name == name) {
                return &p;
            }
        }

        return nullptr;
    }

    ply_element() : name(""), count(0) {}
};

typedef uint16_t ply_version;

#define MAKE_PLY_VERSION(maj,min) ((uint16_t)((maj) << 8) | (uint16_t)(min))
#define PLY_SUCCESS (0)

struct ply_mesh {
    ply_version version;
    ply_format format;
    std::vector<ply_element> elements;
};

#include <assert.h>

static void ply_read_element_data(std::ifstream &ios, ply_element &e)
{
    for (auto it = e.props.begin(); it != e.props.end(); ++it) {
        if (!it->is_list) {
            it->storage.prealloc(e.count * ply_get_type_size(it->type));
        }
    }

    for (int i = 0; i < e.count; ++i) {
        // read line of properties
        std::string line;
        std::getline(ios, line);
        std::stringstream ss(line);

        for (auto p = e.props.begin(); p != e.props.end(); ++p) {
            size_t read_count = 1;

            if (p->is_list) {
                ss >> read_count;
            }

            while (read_count != 0) {
                switch (p->type) {
                    default:break;
                    case PLY_TYPE_FLOAT: {
                    float v;
                    ss >> v;
                    p->storage.add_val<float>(v);
                    break;
                    }
                    case PLY_TYPE_UINT:case PLY_TYPE_INT: {
                    int v;
                    ss >> v;
                    p->storage.add_val<int>(v);
                    break;
                    }
                }

                --read_count;
            }
            
        }
    }
}

static int load_ply_mesh(ply_mesh &mesh, const char *filepath)
{
    uint32_t face_count, vertex_count;
    std::string keyword;
    float x,y,z;
    std::ifstream ios;
    ios.open(filepath, std::ios_base::binary | std::ios_base::in);
    if (!ios.is_open()) {
        return -1; // failed to open file
    }

    std::string line;
    std::getline(ios,line);
    if (line != "ply") {
        return -1;
    }

    // parse format
    std::getline(ios, line);
    std::stringstream lns(line);
    std::string format,version;
    
    lns >> keyword >> format >> version;

    assert(keyword == "format");

    if (format == "ascii") {
        mesh.format = PLY_ASCII;
    } else if (format == "binary_little_endian") {
        mesh.format = PLY_BINARY_LITTLE_ENDIAN;
    } else if (format == "binary_big_endian") {
        mesh.format = PLY_BINARY_BIG_ENDIAN;
    }

    assert(version == "1.0");
    mesh.version = MAKE_PLY_VERSION(1,0);

    int elem_index = -1;

    for (;;) {
        std::getline(ios, line);
        lns = std::stringstream(line);
        lns >> keyword;

        if (keyword == "comment") {
            // skip line
            continue;
        } else if (keyword == "end_header") {
            // end of header
            break;
        } else if (keyword == "property") {
            ply_property property;
            std::string base_type;

            if (elem_index == -1) {
                // error, encountered property line with no corresponding element line coming prior
                exit(-1);
            }

            lns >> base_type;
            if (base_type == "list") {
                property.is_list = true;
                std::string count_type, var_type;

                lns >> count_type >> var_type;
                property.count_type = ply_get_token_type(count_type);
                property.type = ply_get_token_type(var_type);
            } else {
                property.is_list = false;
                property.type = ply_get_token_type(base_type);
            }

            lns >> property.name;
            mesh.elements[elem_index].props.push_back(property);
        } else if (keyword == "element") {
            std::string element_type;
            ply_element elem;
            size_t number;
            lns >> element_type >> number;
            elem.name = element_type;
            elem.count = number;
            elem_index = mesh.elements.size();
            mesh.elements.push_back(elem);
        }
    }
   
    // read vertex list
    auto &e_vertex = mesh.elements[0]; // vertex
    ply_read_element_data(ios,e_vertex);

    // read face list
    auto &e_face = mesh.elements[1]; // face
    ply_read_element_data(ios,e_face);

    ios.close();
    return PLY_SUCCESS;
}

#endif // MODEL_H