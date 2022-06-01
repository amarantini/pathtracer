//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Material.hpp"
#include <time.h>
#include "pcg-cpp-0.98/include/pcg_random.hpp"
#include <fstream>

void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const {
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const {
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()) {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum) {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object *> &objects,
        float &tNear, uint32_t &index, Object **hitObject) {
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

bool compare_Vector3f(Vector3f x, Vector3f y, float epsilon = EPSILON){
    if(fabs(x.x - y.x) < epsilon && fabs(x.y - y.y) < epsilon && fabs(x.z - y.z) < epsilon) {
        return true; //they are same
    }
    return false; //they are not same
}

bool compare_float(float x, float y, float epsilon = EPSILON) {
    return fabs(x-y)<epsilon;
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray) const {
    Intersection its = intersect(ray);
    if (!its.happened) return Vector3f(0.f);
    
    Vector3f N = normalize(its.normal);
    Vector3f wo = normalize(-ray.direction);

    //If is a light source, return L_i directly
    if(its.obj->hasEmit())
        return its.emit; 
    
    //Direct illumination (light sampling)
    Vector3f L_dir = Vector3f(0.f);
    Intersection light_its = Intersection();
    float pdf = 0.0f;
    sampleLight(light_its,pdf);
    Vector3f wi = normalize(light_its.coords-its.coords);//origin at shading point
    Ray light_ray = Ray(its.coords,wi);
    Intersection shadowCheck_its = intersect(light_ray);
    if(shadowCheck_its.happened && shadowCheck_its.obj->hasEmit()){ //Light ray not blocked
        //Calculate L_dir
        Vector3f fr = its.m->eval(wi,wo,N);
        float cosine = dotProduct(N,wi);
        float cosine_p = dotProduct(light_its.normal,-wi);
        if(cosine_p>0.0f && cosine>0.0f) {
            pdf = pdf * pow((light_its.coords-its.coords).norm(),2) / cosine_p;//transform pdf from area to solid angle
            L_dir = light_its.emit * fr * cosine / pdf;
        }
        
    }

    //Indirect illumination (BRDF sampling)
    Vector3f L_indir = Vector3f(0.f);
    //Russian Roulette
    pcg_extras::seed_seq_from<std::random_device> seed_source;
    pcg32 rng(seed_source);
    std::uniform_real_distribution<double> distribution(0.0,1.0);
    double ksi = distribution(rng);
    if(ksi > RussianRoulette)
        return L_dir;
    Vector3f wi_brdf = normalize(its.m->sample(-wo,N)); //Origin at shading point
    Ray new_ray = Ray(its.coords, wi_brdf);
    Intersection new_its = intersect(new_ray); 
    if(new_its.happened && !new_its.obj->hasEmit()) { //If ray do not hit the light
        Vector3f fr = its.m->eval(wi_brdf,wo,N);
        float cosine = dotProduct(N,wi_brdf);
        float brdf_pdf = its.m->pdf(wi_brdf,wo,N);
        if(cosine>0.0f)
            L_indir =  castRay(new_ray) * fr * cosine / brdf_pdf / RussianRoulette;
    }

    return L_dir + L_indir;
   
}




// MIS Implementation of Path Tracing
// Vector3f Scene::castRay(const Ray &ray) const {
//     Intersection its = intersect(ray);
//     if (!its.happened) return Vector3f(0.f);
    
//     Vector3f N = normalize(its.normal);
//     Vector3f wo = normalize(-ray.direction);

//     //If is a light source, return L_i directly
//     if(its.obj->hasEmit())
//         return its.emit; 


//     Vector3f L_indir = Vector3f(0.f);
//     Vector3f L_dir = Vector3f(0.f);

//     //Direct illumination 
//     //light sampling
//     Intersection light_its = Intersection();
//     float pdf = 0.0f;
//     sampleLight(light_its,pdf);
//     Vector3f wi = normalize(light_its.coords-its.coords);//origin at shading point
//     Ray light_ray = Ray(its.coords,wi);
//     Intersection shadowCheck_its = intersect(light_ray);  //check if the light ray is blocked
//     float cosine_p = dotProduct(light_its.normal,-wi);
//     pdf = pdf * pow((light_its.coords-its.coords).norm(),2) / cosine_p;//transform pdf from area to solid angle
//     //BRDF sampling
//     Vector3f wi_brdf = normalize(its.m->sample(-wo,N)); //Origin at shading point
//     float brdf_pdf = its.m->pdf(wi_brdf,wo,N);
//     Ray new_ray = Ray(its.coords, wi_brdf);
//     Intersection new_its = intersect(new_ray);

//     //Calculate L_dir
//     if(shadowCheck_its.happened && shadowCheck_its.obj->hasEmit()){//Light visible
        
//         Vector3f fr = its.m->eval(wi,wo,N);
//         float cosine = dotProduct(N,wi);  
//         if(cosine>0.0f && cosine_p>0.0f)
//              L_dir = light_its.emit * fr * cosine / pdf * pdf / (pdf+brdf_pdf); 
//     }
//     if(new_its.happened && new_its.obj->hasEmit()) { //BRDF
//         Vector3f fr = its.m->eval(wi_brdf,wo,N);
//         float cosine = dotProduct(N,wi_brdf);
//         if(cosine>0.0f)
//              L_dir +=  new_its.emit * fr * cosine / brdf_pdf * brdf_pdf / (pdf+brdf_pdf);
//     }

//     //Indirect illumination
//     //Russian Roulette
//     pcg_extras::seed_seq_from<std::random_device> seed_source;
//     pcg32 rng(seed_source);
//     std::uniform_real_distribution<double> distribution(0.0,1.0);
//     double ksi = distribution(rng);
//     if(ksi > RussianRoulette) 
//         return L_dir;

//     //BRDF sampling
//     if(new_its.happened && !new_its.obj->hasEmit()) { //If ray do not hit the light
//         Vector3f fr = its.m->eval(wi_brdf,wo,N);
//         float cosine = dotProduct(N,wi_brdf);
//         if (cosine>0.0f)
//             L_indir =  castRay(new_ray) * fr * cosine / brdf_pdf / RussianRoulette;
//     }

//     return L_dir + L_indir;
// }