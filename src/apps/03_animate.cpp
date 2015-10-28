#include "scene.h"
#include "image.h"
#include "tesselation.h"
#include "gls.h"
#include "fstream"
#include <cstdio>


std::ofstream fout;
void uiloop();

string scene_filename;          // scene filename
string image_filename;          // image filename
Scene* scene;                   // scene


// get keyframe interval that contains time
pair<int,float> get_keyframe_details(const vector<int> &keytimes, int time) {
    auto interval = 0;
    auto t = 0.0f;
    
    put_your_code_here("Implement keyframe details function");
    
    // find interval of keytime where keytimes[interval] < time <= keytimes[interval+1]
    // compute t
    int start = 0, end = keytimes.size()-1,mid;
    while(start <= end){
        mid = (start + end ) /2;
        if(keytimes[mid] < time && time <= keytimes[mid+1])
            break;
        else if(time > keytimes[mid])
            start = mid+1;
        else
            end = mid-1;
    }
    t = double(time - keytimes[mid]) / double(keytimes[mid+1] - keytimes[mid]);
    interval = mid;
    return make_pair(interval,t);
}

// compute the frame from an animation
frame3f animate_compute_frame(FrameAnimation* animation, int time) {
    // find keyframe interval and t
    auto interval_t = get_keyframe_details(animation->keytimes, time);
    auto interval   = interval_t.first;
    auto t          = interval_t.second;
    put_your_code_here("Implement compute frame function");
    //translation
    // get translation and rotation matrices
    vec3f translate_frame = (1-t) * vec3f(animation->translation[interval]) + t * vec3f(animation->translation[interval+1]);
    vec3f rotate_frame = (1-t) * vec3f(animation->rotation[interval]) + t * vec3f(animation->rotation[interval+1]);

    // compute combined xform matrix
    mat4f translate_matrix = translation_matrix(translate_frame);
    mat4f rotate_z_matrix = rotation_matrix(rotate_frame.z, z3f);
    mat4f rotate_y_matrix = rotation_matrix(rotate_frame.y, y3f);
    mat4f rotate_x_matrix = rotation_matrix(rotate_frame.x, x3f);
    mat4f final_matrix = translate_matrix * rotate_z_matrix * rotate_y_matrix * rotate_x_matrix;

    // return the transformed rest frame
    return transform_frame(final_matrix, animation->rest_frame);
}

// update mesh frames for animation
void animate_frame(Scene* scene) {
    
    put_your_code_here("Implement frame animation");
    for(int i = 0 ; i < scene->meshes.size() ; ++i){
          if(scene->meshes[i]->animation == nullptr)  continue;
          scene->meshes[i]->frame = animate_compute_frame(scene->meshes[i]->animation, scene->animation->time);
    }
    for(int i = 0 ; i < scene->surfaces.size(); ++i){
        if(scene->surfaces[i]->animation == nullptr)    continue;
        scene->surfaces[i]->frame = animate_compute_frame(scene->surfaces[i]->animation, scene->animation->time);
        if(scene->surfaces[i]->_display_mesh == nullptr)  continue;
        scene->surfaces[i]->_display_mesh->frame = scene->surfaces[i]->frame;
    }
    // foreach mesh
        // if not animation, continue
        // call animate_compute_frame and update mesh frame
    // foreach surface
        // if not animation, continue
        // call animate_compute_frame and update surface frame
        // update the _display_mesh frame if exists
}

// skinning scene
void animate_skin(Scene* scene) {
    
    put_your_code_here("Implement skin animation");

    for(int i = 0 ; i < scene->meshes.size() ; ++i){
        MeshSkinning* skinning = scene->meshes[i]->skinning;
        if(skinning == nullptr)   continue;
        for(int j = 0 ; j < scene->meshes[i]->pos.size() ; ++j){
            vec3f pos = zero3f;
            vec3f norm = zero3f;
            for(int m = 0 ; m < 4 ; ++m){
                float weight = skinning->bone_weights[j][m];
                int index = skinning->bone_ids[j][m];
                if(index < 0)   continue;
                mat4f xform = skinning->bone_xforms[scene->animation->time][index];
                pos += weight * transform_point(xform,skinning->rest_pos[j]);
                norm += weight  * transform_normal(xform,skinning->rest_norm[j]);
            }
            normalize(norm);
            scene->meshes[i]->pos[j] = vec3f(pos.x,pos.y,pos.z);
            scene->meshes[i]->norm[j] = vec3f(norm.x,norm.y,norm.z);
        }
    }
    // foreach mesh
        // if no skinning, continue
        // foreach vertex index
            // set pos/norm to zero
            // for each bone slot (0..3)
                // get bone weight and index
                // if index < 0, continue
                // grab bone xform
                // accumulate weighted and transformed rest position and normal
            // normalize normal
}

// particle simulation
void simulate(Scene* scene) {
    
    put_your_code_here("Implement simulation");
    
    // for each mesh
    for(Mesh* mesh : scene->meshes){
        // skip if no simulation
        if(mesh->simulation == nullptr) continue;
        // compute time per step
        float time_step = scene->animation->dt / scene->animation->simsteps;
        // foreach simulation steps
        for(int i = 0 ; i < scene->animation->simsteps ; ++i){
            // initialize particle forces to zero3f
            for(int j = 0 ; j < mesh->simulation->force.size() ; ++j)
                mesh->simulation->force[j] = zero3f;
            // foreach particle, compute external forces
            for(int particle_index = 0 ; particle_index < mesh->pos.size() ; ++particle_index){
                // compute force of gravity
                vec3f gravity_force = scene->animation->gravity * mesh->simulation->mass[particle_index];
                // compute force of wind
                vec3f wind_force = zero3f;
                // accumulate sum of forces on particle
                mesh->simulation->force[particle_index] = gravity_force + wind_force;
            }

            // for each spring, compute spring force on points
            for(int spring_index = 0 ; spring_index < mesh->simulation->springs.size() ; ++spring_index){
                // compute spring distance and length
                int particle_index_1 = mesh->simulation->springs[spring_index].ids.x;
                int particle_index_2 = mesh->simulation->springs[spring_index].ids.y;
                float distance = dist(mesh->pos[particle_index_1],mesh->pos[particle_index_2]);
                float length = mesh->simulation->springs[spring_index].restlength;
                vec3f norm = mesh->pos[particle_index_1] - mesh->pos[particle_index_2];
                // compute static force
                vec3f static_force = mesh->simulation->springs[spring_index].ks * (distance - length) * norm / distance;
                // accumulate static force on points
                mesh->simulation->force[particle_index_1] -= static_force;
                mesh->simulation->force[particle_index_2] += static_force;
                // compute dynamic force
                float velocity = dot(mesh->simulation->vel[particle_index_1], norm/distance) - dot(mesh->simulation->vel[particle_index_2], norm/distance);
                vec3f dynamic_force = mesh->simulation->springs[spring_index].kd * velocity * norm / distance;
                // accumulate dynamic force on points
                mesh->simulation->force[particle_index_1] -= dynamic_force;
                mesh->simulation->force[particle_index_2] += dynamic_force;
            }

            // foreach particle, integrate using newton laws
            for(int particle_index = 0 ; particle_index < mesh->pos.size() ; ++particle_index){
                // if pinned, skip
                if(mesh->simulation->pinned[particle_index])    continue;
                // compute acceleration
                vec3f accelaration = mesh->simulation->force[particle_index] / mesh->simulation->mass[particle_index];
                // update velocity and positions using Euler's method
                mesh->pos[particle_index] +=  mesh->simulation->vel[particle_index] * time_step + accelaration * time_step * time_step / 2;
                mesh->simulation->vel[particle_index] += accelaration * time_step;
                // for each surface, check for collision
                for(Surface* surface : scene->surfaces){
                    // compute inside tests...
                    // if quad
                    if(surface->isquad){
                        // compute local position
                        vec3f local_pos = transform_point_to_local(surface->frame, mesh->pos[particle_index]);
                        vec3f local_vel = transform_point_to_local(surface->frame, mesh->simulation->vel[particle_index]);
                        // perform inside test
                        if(fabsf(local_pos.x) < surface->radius && fabsf(local_pos.y) < surface->radius && local_pos.z < 0){
                            // if inside, compute a collision position and normal
                            float time = dot(zero3f - local_pos, z3f) / dot(local_vel,z3f);
                            local_pos += time * local_vel;
                            local_vel.x *= (1 - scene->animation->bounce_dump.x);
                            local_vel.y *= (1 - scene->animation->bounce_dump.x);
                            local_vel.z = -local_vel.z * (1 - scene->animation->bounce_dump.y);

                            // set particle position
                            // update velocity (particle bounces), taking into account loss of kinetic energy
                            mesh->pos[particle_index] = transform_point_from_local(surface->frame, local_pos);
                            mesh->simulation->vel[particle_index] = transform_vector_from_local(surface->frame, local_vel);
                        }
                    }else{
                    // if sphere
                        float distance = dist(mesh->pos[particle_index], surface->frame.o);
                        // inside test
                        if(distance < surface->radius){
                            // if inside, compute a collision position and normal
                            vec3f collision_pos = surface->frame.o + surface->radius * normalize(mesh->pos[particle_index] - surface->frame.o);
                            vec3f collision_norm = normalize(mesh->pos[particle_index] - surface->frame.o);
                            vec3f vel = mesh->simulation->vel[particle_index] - 2 * collision_norm * dot(collision_norm, mesh->simulation->vel[particle_index]);
                            frame3f collision_frame = frame_from_z(collision_norm);
                            vec3f local_vel = transform_vector_to_local(collision_frame, vel);
                            local_vel.x *= (1 - scene->animation->bounce_dump.x);
                            local_vel.y *= (1 - scene->animation->bounce_dump.x);
                            local_vel.z *= (1 - scene->animation->bounce_dump.y);

                            // set particle position
                            // update velocity (particle bounces), taking into account loss of kinetic energy
                            mesh->simulation->vel[particle_index] = transform_vector_from_local(collision_frame, local_vel);
                            mesh->pos[particle_index] = collision_pos;
                        }
                    }
                }
            }
        }
        // smooth normals if it has triangles or quads
    }
}

// scene reset
void animate_reset(Scene* scene) {
    scene->animation->time = 0;
    for(auto mesh : scene->meshes) {
        if(mesh->animation) {
            mesh->frame = mesh->animation->rest_frame;
        }
        if(mesh->skinning) {
            mesh->pos = mesh->skinning->rest_pos;
            mesh->norm = mesh->skinning->rest_norm;
        }
        if(mesh->simulation) {
            mesh->pos = mesh->simulation->init_pos;
            mesh->simulation->vel = mesh->simulation->init_vel;
            mesh->simulation->force.resize(mesh->simulation->init_pos.size());
        }
    }
}

// scene update
void animate_update(Scene* scene, bool skinning_gpu) {
    scene->animation->time ++;
    if(scene->animation->time >= scene->animation->length) animate_reset(scene);
    animate_frame(scene);
    if(not skinning_gpu) animate_skin(scene);
    simulate(scene);
}




// main function
int main(int argc, char** argv) { fout.open("data.txt");
    auto args = parse_cmdline(argc, argv,
        { "03_animate", "view scene",
            {  {"resolution", "r", "image resolution", typeid(int), true, jsonvalue() }  },
            {  {"scene_filename", "", "scene filename", typeid(string), false, jsonvalue("scene.json")},
               {"image_filename", "", "image filename", typeid(string), true, jsonvalue("")}  }
        });
    
    // generate/load scene either by creating a test scene or loading from json file
    scene_filename = args.object_element("scene_filename").as_string();
    scene = nullptr;
    if(scene_filename.length() > 9 and scene_filename.substr(0,9) == "testscene") {
        int scene_type = atoi(scene_filename.substr(9).c_str());
        scene = create_test_scene(scene_type);
        scene_filename = scene_filename + ".json";
    } else {
        scene = load_json_scene(scene_filename);
    }
    error_if_not(scene, "scene is nullptr");
    
    image_filename = (args.object_element("image_filename").as_string() != "") ?
        args.object_element("image_filename").as_string() :
        scene_filename.substr(0,scene_filename.size()-5)+".png";
    
    if(not args.object_element("resolution").is_null()) {
        scene->image_height = args.object_element("resolution").as_int();
        scene->image_width = scene->camera->width * scene->image_height / scene->camera->height;
    }
    
    animate_reset(scene);
    
    subdivide(scene);
    
    uiloop();
}





/////////////////////////////////////////////////////////////////////
// UI and Rendering Code: OpenGL, GLFW, GLSL


bool save         = false;      // whether to start the save loop
bool animate      = true;      // run animation
bool draw_faces   = true;       // draw faces of mesh
bool draw_lines   = true;       // draw lines/splines of mesh
bool draw_points  = true;       // draw points of mesh
bool draw_edges   = false;      // draw edges of mesh
bool draw_normals = false;      // draw normals

bool skinning_gpu = false;      // skinning on the gpu    NOTE: NOT USED!

int gl_program_id = 0;          // OpenGL program handle
int gl_vertex_shader_id = 0;    // OpenGL vertex shader handle
int gl_fragment_shader_id = 0;  // OpenGL fragment shader handle
map<image3f*,int> gl_texture_id;// OpenGL texture handles

// initialize the shaders
void init_shaders() {
    // load shader code from files
    auto vertex_shader_code    = load_text_file("animate_vertex.glsl");
    auto fragment_shader_code  = load_text_file("animate_fragment.glsl");
    auto vertex_shader_codes   = (char *)vertex_shader_code.c_str();
    auto fragment_shader_codes = (char *)fragment_shader_code.c_str();

    // create shaders
    gl_vertex_shader_id = glCreateShader(GL_VERTEX_SHADER);
    gl_fragment_shader_id = glCreateShader(GL_FRAGMENT_SHADER);
    
    // load shaders code onto the GPU
    glShaderSource(gl_vertex_shader_id,1,(const char**)&vertex_shader_codes,nullptr);
    glShaderSource(gl_fragment_shader_id,1,(const char**)&fragment_shader_codes,nullptr);
    
    // compile shaders
    glCompileShader(gl_vertex_shader_id);
    glCompileShader(gl_fragment_shader_id);
    
    // check if shaders are valid
    error_if_glerror();
    error_if_shader_not_valid(gl_vertex_shader_id);
    error_if_shader_not_valid(gl_fragment_shader_id);
    
    // create program
    gl_program_id = glCreateProgram();
    
    // attach shaders
    glAttachShader(gl_program_id,gl_vertex_shader_id);
    glAttachShader(gl_program_id,gl_fragment_shader_id);
    
    // bind vertex attributes locations
    glBindAttribLocation(gl_program_id, 0, "vertex_pos");
    glBindAttribLocation(gl_program_id, 1, "vertex_norm");
    glBindAttribLocation(gl_program_id, 2, "vertex_texcoord");
    glBindAttribLocation(gl_program_id, 3, "vertex_skin_bones");
    glBindAttribLocation(gl_program_id, 4, "vertex_skin_weights");

    // link program
    glLinkProgram(gl_program_id);
    
    // check if program is valid
    error_if_glerror();
    error_if_program_not_valid(gl_program_id);
}

// initialize the textures
void init_textures(Scene* scene) {
    // grab textures from scene
    auto textures = get_textures(scene);
    // foreach texture
    for(auto texture : textures) {
        // if already in the gl_texture_id map, skip
        if(gl_texture_id.find(texture) != gl_texture_id.end()) continue;
        // gen texture id
        unsigned int id = 0;
        glGenTextures(1, &id);
        // set id to the gl_texture_id map for later use
        gl_texture_id[texture] = id;
        // bind texture
        glBindTexture(GL_TEXTURE_2D, id);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP, GL_TRUE);
        // load texture data
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
                     texture->width(), texture->height(),
                     0, GL_RGB, GL_FLOAT, texture->data());
    }
}

// utility to bind texture parameters for shaders
// uses texture name, texture_on name, texture pointer and texture unit position
void _bind_texture(string name_map, string name_on, image3f* txt, int pos) {
    // if txt is not null
    if(txt) {
        // set texture on boolean parameter to true
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_TRUE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // bind texture object to it from gl_texture_id map
        glBindTexture(GL_TEXTURE_2D, gl_texture_id[txt]);
        // set texture parameter to the position pos
        glUniform1i(glGetUniformLocation(gl_program_id, name_map.c_str()), pos);
    } else {
        // set texture on boolean parameter to false
        glUniform1i(glGetUniformLocation(gl_program_id,name_on.c_str()),GL_FALSE);
        // activate a texture unit at position pos
        glActiveTexture(GL_TEXTURE0+pos);
        // set zero as the texture id
        glBindTexture(GL_TEXTURE_2D, 0);
    }
}

// shade a mesh
void shade_mesh(Mesh* mesh, int time) {
    // bind material kd, ks, n
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_kd"),
                 1,&mesh->mat->kd.x);
    glUniform3fv(glGetUniformLocation(gl_program_id,"material_ks"),
                 1,&mesh->mat->ks.x);
    glUniform1f(glGetUniformLocation(gl_program_id,"material_n"),
                mesh->mat->n);
    glUniform1i(glGetUniformLocation(gl_program_id,"material_is_lines"),
                GL_FALSE);
    glUniform1i(glGetUniformLocation(gl_program_id,"material_double_sided"),
                (mesh->mat->double_sided)?GL_TRUE:GL_FALSE);
    // bind texture params (txt_on, sampler)
    _bind_texture("material_kd_txt", "material_kd_txt_on", mesh->mat->kd_txt, 0);
    _bind_texture("material_ks_txt", "material_ks_txt_on", mesh->mat->ks_txt, 1);
    _bind_texture("material_norm_txt", "material_norm_txt_on", mesh->mat->norm_txt, 2);
    
    // bind mesh frame - use frame_to_matrix
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"mesh_frame"),
                       1,true,&frame_to_matrix(mesh->frame)[0][0]);
    
    // enable vertex attributes arrays and set up pointers to the mesh data
    auto vertex_pos_location = glGetAttribLocation(gl_program_id, "vertex_pos");
    auto vertex_norm_location = glGetAttribLocation(gl_program_id, "vertex_norm");
    auto vertex_texcoord_location = glGetAttribLocation(gl_program_id, "vertex_texcoord");
    auto vertex_skin_bone_ids_location = glGetAttribLocation(gl_program_id, "vertex_skin_bone_ids");
    auto vertex_skin_bone_weights_location = glGetAttribLocation(gl_program_id, "vertex_skin_bone_weights");
    
    glEnableVertexAttribArray(vertex_pos_location);
    glVertexAttribPointer(vertex_pos_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->pos[0].x);
    glEnableVertexAttribArray(vertex_norm_location);
    glVertexAttribPointer(vertex_norm_location, 3, GL_FLOAT, GL_FALSE, 0, &mesh->norm[0].x);
    if(not mesh->texcoord.empty()) {
        glEnableVertexAttribArray(vertex_texcoord_location);
        glVertexAttribPointer(vertex_texcoord_location, 2, GL_FLOAT, GL_FALSE, 0, &mesh->texcoord[0].x);
    }
    else glVertexAttrib2f(vertex_texcoord_location, 0, 0);
    
    if (mesh->skinning and skinning_gpu) {
        glUniform1i(glGetUniformLocation(gl_program_id,"skinning->enabled"),GL_TRUE);
        glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"skinning->bone_xforms"),
                           mesh->skinning->bone_xforms[time].size(), GL_TRUE,
                           &mesh->skinning->bone_xforms[time][0].x.x);
        glEnableVertexAttribArray(vertex_skin_bone_ids_location);
        glEnableVertexAttribArray(vertex_skin_bone_weights_location);
        glVertexAttribPointer(vertex_skin_bone_ids_location, 4, GL_INT, GL_FALSE, 0, mesh->skinning->bone_ids.data());
        glVertexAttribPointer(vertex_skin_bone_weights_location, 4, GL_FLOAT, GL_FALSE, 0, mesh->skinning->bone_weights.data());
    } else {
        glUniform1i(glGetUniformLocation(gl_program_id,"skinning->enabled"),GL_FALSE);
    }
    
    // draw triangles and quads
    if(draw_faces) {
        if(mesh->triangle.size()) glDrawElements(GL_TRIANGLES, mesh->triangle.size()*3, GL_UNSIGNED_INT, &mesh->triangle[0].x);
        if(mesh->quad.size())     glDrawElements(GL_QUADS, mesh->quad.size()*4, GL_UNSIGNED_INT, &mesh->quad[0].x);
    }
    
    if(draw_points) {
        if(mesh->point.size()) glDrawElements(GL_POINTS, mesh->point.size(), GL_UNSIGNED_INT, &mesh->point[0]);
    }
    
    if(draw_lines) {
        if(mesh->line.size()) glDrawElements(GL_LINES, mesh->line.size(), GL_UNSIGNED_INT, &mesh->line[0].x);
        for(auto segment : mesh->spline) glDrawElements(GL_LINE_STRIP, 4, GL_UNSIGNED_INT, &segment);
    }
    
    if(draw_edges) {
        auto edges = EdgeMap(mesh->triangle, mesh->quad).edges();
        glDrawElements(GL_LINES, edges.size()*2, GL_UNSIGNED_INT, &edges[0].x);
    }
    
    // disable vertex attribute arrays
    glDisableVertexAttribArray(vertex_pos_location);
    glDisableVertexAttribArray(vertex_norm_location);
    if(not mesh->texcoord.empty()) glDisableVertexAttribArray(vertex_texcoord_location);
    if(mesh->skinning) {
        glDisableVertexAttribArray(vertex_skin_bone_ids_location);
        glDisableVertexAttribArray(vertex_skin_bone_weights_location);
    }
    
    // draw normals if needed
    if(draw_normals) {
        glUniform3fv(glGetUniformLocation(gl_program_id,"material_kd"),
                     1,&zero3f.x);
        glUniform3fv(glGetUniformLocation(gl_program_id,"material_ks"),
                     1,&zero3f.x);
        glBegin(GL_LINES);
        for(auto i : range(mesh->pos.size())) {
            auto p0 = mesh->pos[i];
            auto p1 = mesh->pos[i] + mesh->norm[i]*0.1;
            glVertexAttrib3fv(0,&p0.x);
            glVertexAttrib3fv(0,&p1.x);
            if(mesh->mat->double_sided) {
                auto p2 = mesh->pos[i] - mesh->norm[i]*0.1;
                glVertexAttrib3fv(0,&p0.x);
                glVertexAttrib3fv(0,&p2.x);
            }
        }
        glEnd();
    }
}

// render the scene with OpenGL
void shade(Scene* scene) {
    // enable depth test
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    // disable culling face
    glDisable(GL_CULL_FACE);
    // let the shader control the points
    glEnable(GL_POINT_SPRITE);
    
    // set up the viewport from the scene image size
    glViewport(0, 0, scene->image_width, scene->image_height);
    
    // clear the screen (both color and depth) - set cleared color to background
    glClearColor(scene->background.x, scene->background.y, scene->background.z, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // enable program
    glUseProgram(gl_program_id);
    
    // bind camera's position, inverse of frame and projection
    // use frame_to_matrix_inverse and frustum_matrix
    glUniform3fv(glGetUniformLocation(gl_program_id,"camera_pos"),
                 1, &scene->camera->frame.o.x);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_frame_inverse"),
                       1, true, &frame_to_matrix_inverse(scene->camera->frame)[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(gl_program_id,"camera_projection"),
                       1, true, &frustum_matrix(-scene->camera->dist*scene->camera->width/2, scene->camera->dist*scene->camera->width/2,
                                                -scene->camera->dist*scene->camera->height/2, scene->camera->dist*scene->camera->height/2,
                                                scene->camera->dist,10000)[0][0]);
    
    // bind ambient and number of lights
    glUniform3fv(glGetUniformLocation(gl_program_id,"ambient"),1,&scene->ambient.x);
    glUniform1i(glGetUniformLocation(gl_program_id,"lights_num"),scene->lights.size());
    
    // foreach light
    auto count = 0;
    for(auto light : scene->lights) {
        // bind light position and internsity (create param name with tostring)
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_pos[%d]",count).c_str()),
                     1, &light->frame.o.x);
        glUniform3fv(glGetUniformLocation(gl_program_id,tostring("light_intensity[%d]",count).c_str()),
                     1, &light->intensity.x);
        count++;
    }
    
    // foreach mesh
    for(auto mesh : scene->meshes) {
        // draw mesh
        shade_mesh(mesh, scene->animation->time);
    }
    
    // foreach surface
    for(auto surface : scene->surfaces) {
        // draw display mesh
        shade_mesh(surface->_display_mesh, scene->animation->time);
    }
}


// uiloop
void uiloop() {
    
    auto ok_glfw = glfwInit();
    error_if_not(ok_glfw, "glfw init error");
    
    // setting an error callback
    glfwSetErrorCallback([](int ecode, const char* msg){ return error(msg); });
    
    glfwWindowHint(GLFW_SAMPLES, scene->image_samples);

    auto window = glfwCreateWindow(scene->image_width, scene->image_height,
                                   "graphics | animate", NULL, NULL);
    error_if_not(window, "glfw window error");
    
    glfwMakeContextCurrent(window);
    
    glfwSetCharCallback(window, [](GLFWwindow* window, unsigned int key) {
        switch (key) {
            case 's': { save = true; } break;
            case ' ': { animate = not animate; } break;
            case '.': { animate_update(scene, skinning_gpu); } break;
            case 'g': { skinning_gpu = not skinning_gpu; animate_reset(scene); } break;
            case 'n': { draw_normals = not draw_normals; } break;
            case 'e': { draw_edges = not draw_edges; } break;
            case 'p': { draw_points = not draw_points; } break;
            case 'f': { draw_faces = not draw_faces; } break;
        }
    });
    
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    
    auto ok_glew = glewInit();
    error_if_not(GLEW_OK == ok_glew, "glew init error");
    
    init_shaders();
    init_textures(scene);
    animate_reset(scene);
    
    auto mouse_last_x = -1.0;
    auto mouse_last_y = -1.0;
    
    auto last_update_time = glfwGetTime();
    
    while(not glfwWindowShouldClose(window)) {
        auto title = tostring("graphics | animate | %03d", scene->animation->time);
        glfwSetWindowTitle(window, title.c_str());

        if(animate) {
            if(glfwGetTime() - last_update_time > scene->animation->dt) {
                last_update_time = glfwGetTime();
                animate_update(scene, skinning_gpu);
            }
        }
        
        if(save) {
            animate_reset(scene);
            for(auto i : range(scene->animation->length/3)) animate_update(scene, skinning_gpu);
        }
        
        glfwGetFramebufferSize(window, &scene->image_width, &scene->image_height);
        scene->camera->width = (scene->camera->height * scene->image_width) / scene->image_height;
        
        shade(scene);

        if(glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)) {
            double x, y;
            glfwGetCursorPos(window, &x, &y);
            if (mouse_last_x < 0 or mouse_last_y < 0) { mouse_last_x = x; mouse_last_y = y; }
            auto delta_x = x - mouse_last_x, delta_y = y - mouse_last_y;
            
            set_view_turntable(scene->camera, delta_x*0.01, -delta_y*0.01, 0, 0, 0);
            
            mouse_last_x = x;
            mouse_last_y = y;
        } else { mouse_last_x = -1; mouse_last_y = -1; }
        
        if(save) {
            auto image = image3f(scene->image_width,scene->image_height);
            glReadPixels(0, 0, scene->image_width, scene->image_height, GL_RGB, GL_FLOAT, &image.at(0,0));
            write_png(image_filename, image, true);
            save = false;
        }
        
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwDestroyWindow(window);
    
    glfwTerminate();
}


