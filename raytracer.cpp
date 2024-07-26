#include <iostream>
#include "parser.h"
#include "ppm.h"
#include <math.h>
//using namespace std;
using namespace parser;
typedef unsigned char RGB[3];

typedef struct Ray
{
    Vec3f origin;
    Vec3f direction;
    int depth;
} Ray;

typedef struct HitRecord
{
    Material material;
    Vec3f position;
    Vec3f normal;
} HitRecord;

Ray generateRay(Camera cam, int x, int y);
Vec3f ScalarMultiplication (Vec3f a, float b);
Vec3f VectorAddition(Vec3f a, Vec3f b);
Vec3f Normalize (Vec3f a);
Vec3f CrossProduct(Vec3f a, Vec3f b);
float Cosine (Vec3f a, Vec3f b);
float DotProduct(Vec3f a, Vec3f b);
float IntersectSphere(Sphere sphere, Ray ray, Vec3f center);
float IntersectTriangles(Vec3f a, Vec3f b, Vec3f c, Ray ray);
float determinant(Vec3f a, Vec3f b, Vec3f c);
Vec3f Shading(Ray ray , HitRecord hitRecord,std::vector<PointLight> point_lights, Vec3f ambient );
Vec3f SphereNormal(Vec3f point,Vec3f center);
Vec3f TriangleNormal(Vec3f a,Vec3f b, Vec3f c);
Ray ComputeReflection(Ray ray,Vec3f normal);
Vec3f ComputeColor(Ray ray);
parser::Scene scene;

//precomputed variables
Vec3f m;
Vec3f u;
Vec3f q;

std::vector<Vec3f> triangle_normals;
std::vector<std::vector<Vec3f>> mesh_face_normals;
int main(int argc, char* argv[])
{
    // Sample usage for reading an XML scene file
    
    

    scene.loadFromXml(argv[1]);
    // The code below creates a test pattern and writes
    // it to a PPM file to demonstrate the usage of the
    // ppm_write function.
    //
    // Normally, you would be running your ray tracing
    // code here to produce the desired image.

    //precomputing triangle normals
    for(int i=0;i<scene.triangles.size();i++){
        triangle_normals.push_back(TriangleNormal(scene.vertex_data[scene.triangles[i].indices.v0_id-1], scene.vertex_data[scene.triangles[i].indices.v1_id-1], scene.vertex_data[scene.triangles[i].indices.v2_id-1]));
    }
    //precomputing mesh face normals
    
    for(int i=0;i<scene.meshes.size();i++){
        std::vector<Vec3f> temp;
        for(int j = 0;j<scene.meshes[i].faces.size();j++){
            temp.push_back(TriangleNormal(scene.vertex_data[scene.meshes[i].faces[j].v0_id-1], scene.vertex_data[scene.meshes[i].faces[j].v1_id-1], scene.vertex_data[scene.meshes[i].faces[j].v2_id-1]));
        }
        mesh_face_normals.push_back(temp);
    }


    int width,height;
    int columnWidth;
    unsigned char* image;
    
    
    for(int i =0 ; i< scene.cameras.size();i++){
        int p = 0;
        width = scene.cameras[i].image_width, height = scene.cameras[i].image_height;
        columnWidth = width / 8;
        image = new unsigned char [width * height * 3];

        m = VectorAddition(scene.cameras[i].position , ScalarMultiplication(scene.cameras[i].gaze , scene.cameras[i].near_distance));
        u = CrossProduct(scene.cameras[i].up,ScalarMultiplication(scene.cameras[i].gaze,-1));
        q =VectorAddition( VectorAddition(m , ScalarMultiplication(u , scene.cameras[i].near_plane.x)), ScalarMultiplication(scene.cameras[i].up , scene.cameras[i].near_plane.w));

        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                int colIdx = x / columnWidth;
                Vec3f color;
                HitRecord hitRecord;
                
                
                Ray ray = generateRay(scene.cameras[i], x, y);

                color = ComputeColor(ray);

                //color = Shading(ray,hitRecord,scene.point_lights,scene.ambient_light);
                if(color.x>255){
                    color.x=255;
                }
                if(color.y>255){
                    color.y=255;
                }
                if(color.z>255){
                    color.z=255;
                }
                image[p++] = (int)(color.x+0.5f);
                image[p++] = (int)(color.y+0.5f);
                image[p++] = (int)(color.z+0.5f);
                
                
            }
        }

        const char * name = scene.cameras[i].image_name.c_str();
        write_ppm(name, image, width, height);
    }
    

}

Ray generateRay(Camera cam, int x, int y)
{
    Ray ray;

    float su = ((x+0.5) * (cam.near_plane.y - cam.near_plane.x))/cam.image_width;
    float sv = ((y+0.5) * (cam.near_plane.w - cam.near_plane.z))/cam.image_height; 

    Vec3f s = VectorAddition(q , VectorAddition( ScalarMultiplication(u , su) , ScalarMultiplication(cam.up , -sv)));

    ray.origin = cam.position;
    ray.direction = VectorAddition(s, ScalarMultiplication(cam.position,-1));
    ray.direction = Normalize(ray.direction);
    ray.depth = 0;

    return ray;
}

float IntersectSphere(Sphere sphere, Ray ray, Vec3f center)
{
    float t = -1;
    Vec3f L = VectorAddition(ray.origin, ScalarMultiplication(center, -1));
    float a = DotProduct(ray.direction, ray.direction);
    float b = 2 * DotProduct(ray.direction, L);
    float c = DotProduct(L, L) - (sphere.radius * sphere.radius);
    float delta = b * b - 4 * a * c;

    if (delta >= 0)
    {
        float t1 = (-b + sqrt(delta)) / (2 * a);
        float t2 = (-b - sqrt(delta)) / (2 * a);

        if (t1 > scene.shadow_ray_epsilon && t2 > scene.shadow_ray_epsilon)
        {
            t = t1 < t2 ? t1 : t2;
        }
        else if (t1 > scene.shadow_ray_epsilon)
        {
            t = t1;
        }
        else if (t2 > scene.shadow_ray_epsilon)
        {
            t = t2;
        }
    }


    return t;
}

float IntersectTriangles(Vec3f a, Vec3f b, Vec3f c, Ray ray){
    float t = -1;
    Vec3f e1 = VectorAddition(a, ScalarMultiplication(b, -1));
    Vec3f e2 = VectorAddition(a, ScalarMultiplication(c, -1));
    Vec3f s = VectorAddition(a, ScalarMultiplication(ray.origin, -1));
    float det = determinant(e1, e2, ray.direction);
    float dett = determinant(e1, e2, s);

    float t1 = dett / det;
    if(t1>scene.shadow_ray_epsilon){
        float detu = determinant(e1, s, ray.direction);
        float u = detu / det;
        if(u>=0 && u<=1){
            float detv = determinant(s, e2, ray.direction);
            float v = detv / det;
            if(v>=0 && v<=1 && u+v<=1){
                t = t1;
            }
        }
    }
    return t;   
}





Vec3f CrossProduct(Vec3f a, Vec3f b)
{
    Vec3f result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;

    return result;
}

float DotProduct(Vec3f a, Vec3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3f ScalarMultiplication (Vec3f a, float b)
{
    Vec3f result;
    result.x = a.x * b;
    result.y = a.y * b;
    result.z = a.z * b;

    return result;
}

Vec3f VectorAddition(Vec3f a, Vec3f b)
{
    Vec3f result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;

    return result;
}

Vec3f Normalize (Vec3f a){
    Vec3f result;
    float length = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    result.x = a.x / length;
    result.y = a.y / length;
    result.z = a.z / length;

    return result;
}

float Cosine (Vec3f a, Vec3f b){
    float res;
    res = DotProduct(a,b);
    if(res < 0)
    {
        return 0;
    }
    else
    return res;
}

float determinant(Vec3f a, Vec3f b, Vec3f c)
{
    return a.x * (b.y * c.z - b.z * c.y) - a.y * (b.x * c.z - b.z * c.x) + a.z * (b.x * c.y - b.y * c.x);
}   

Vec3f Shading(Ray ray , HitRecord hitRecord, std::vector<PointLight> point_lights, Vec3f ambient){
    Vec3f color;
    color.x = 0;
    color.y = 0;
    color.z = 0;

    if(ray.depth > scene.max_recursion_depth){
        return color;
    }

    //Compute Ambient Color
    
    color.x = hitRecord.material.ambient.x*ambient.x;
    color.y = hitRecord.material.ambient.y*ambient.y;
    color.z = hitRecord.material.ambient.z*ambient.z;
    
    
    //Mirror Reflections
    
     if(hitRecord.material.is_mirror){
        Ray reflection_ray = ComputeReflection(ray,hitRecord.normal);
        reflection_ray.depth = ray.depth + 1;
        reflection_ray.origin =  hitRecord.position;
        //printf("reflection ray data origin %.4f %.4f %.4f  direction %.4f %.4f %.4f\n", reflection_ray.origin.x,reflection_ray.origin.y,reflection_ray.origin.z,reflection_ray.direction.x,reflection_ray.direction.y,reflection_ray.direction.z);
        Vec3f mirror_color = ComputeColor(reflection_ray);
        mirror_color.x = mirror_color.x * hitRecord.material.mirror.x;
        mirror_color.y = mirror_color.y * hitRecord.material.mirror.y;
        mirror_color.z = mirror_color.z * hitRecord.material.mirror.z;
        color = VectorAddition(color,mirror_color);
    }

    Vec3f shadow_start = hitRecord.position;
    shadow_start.x += ScalarMultiplication(hitRecord.normal,scene.shadow_ray_epsilon).x;
    shadow_start.y += ScalarMultiplication(hitRecord.normal,scene.shadow_ray_epsilon).y;
    shadow_start.z += ScalarMultiplication(hitRecord.normal,scene.shadow_ray_epsilon).z;
    //Check for shadows
    for(int i=0;i<point_lights.size();i++){
        Vec3f wi = VectorAddition(point_lights[i].position,ScalarMultiplication(hitRecord.position,-1));
        
        float t = sqrt(wi.x * wi.x + wi.y * wi.y + wi.z * wi.z);
        //printf("to light vector is %.4f %.4f %.4f  with t being %.4f\n",wi.x,wi.y,wi.z,t);
        float rSquared = wi.x*wi.x+wi.y*wi.y+wi.z*wi.z;
        wi = Normalize(wi);
        Ray light_ray;
        light_ray.direction = wi;
        light_ray.origin = shadow_start;
        //printf("shadow ray data origin %.4f %.4f %.4f  direction %.4f %.4f %.4f", light_ray.origin.x,light_ray.origin.y,light_ray.origin.z,light_ray.direction.x,light_ray.direction.y,light_ray.direction.z);
        
        float temp=100000000;

        Vec3f half = Normalize(VectorAddition(ScalarMultiplication(ray.direction,-1),wi));

        for(int j = 0; j < scene.spheres.size(); j++)
        {
            
            float tempt = IntersectSphere(scene.spheres[j], light_ray, scene.vertex_data[scene.spheres[j].center_vertex_id-1]);
            //printf("  distance is %.4f\n",tempt);
            if(tempt > 0 && tempt < temp)
            {
                temp = tempt;
                //printf(" temp %.4f",temp);
            }
            
        }
        for(int j = 0; j < scene.triangles.size(); j++)
        {
            float tempt = IntersectTriangles(scene.vertex_data[scene.triangles[j].indices.v0_id-1], scene.vertex_data[scene.triangles[j].indices.v1_id-1], scene.vertex_data[scene.triangles[j].indices.v2_id-1], light_ray);
            
            if(tempt > 0 && tempt < temp)
            {
                temp = tempt;
            }
            
        }
        
        for(int j = 0; j < scene.meshes.size(); j++)
        {
            for(int k = 0; k < scene.meshes[j].faces.size(); k++)
            {
                float tempt = IntersectTriangles(scene.vertex_data[scene.meshes[j].faces[k].v0_id-1], scene.vertex_data[scene.meshes[j].faces[k].v1_id-1], scene.vertex_data[scene.meshes[j].faces[k].v2_id-1], light_ray);
                if(tempt > 0 && tempt < temp)
                {
                    temp = tempt;
                }
            }
        }


        if(temp<t)
        {
            continue;
        }
        else{
            //Diffuse
            color.x+= hitRecord.material.diffuse.x*Cosine(wi,hitRecord.normal)*(point_lights[i].intensity.x/rSquared);
            color.y+= hitRecord.material.diffuse.y*Cosine(wi,hitRecord.normal)*(point_lights[i].intensity.y/rSquared);
            color.z+= hitRecord.material.diffuse.z*Cosine(wi,hitRecord.normal)*(point_lights[i].intensity.z/rSquared);

            //Specular
            color.x+= hitRecord.material.specular.x*pow(Cosine(hitRecord.normal,half),hitRecord.material.phong_exponent)*(point_lights[i].intensity.x/rSquared);
            color.y+= hitRecord.material.specular.y*pow(Cosine(hitRecord.normal,half),hitRecord.material.phong_exponent)*(point_lights[i].intensity.y/rSquared);
            color.z+= hitRecord.material.specular.z*pow(Cosine(hitRecord.normal,half),hitRecord.material.phong_exponent)*(point_lights[i].intensity.z/rSquared);
        }

    }
    
    
    
    return color;
}   

Vec3f SphereNormal(Vec3f point,Vec3f center){
    Vec3f res = VectorAddition(point,ScalarMultiplication(center,-1));  
    return Normalize(res);
}

Vec3f TriangleNormal(Vec3f a,Vec3f b, Vec3f c){
    Vec3f first = VectorAddition(b,ScalarMultiplication(a,-1));  
    Vec3f second = VectorAddition(c,ScalarMultiplication(b,-1));

    return Normalize(CrossProduct(first,second));
}

Vec3f ComputeColor(Ray ray){

    Vec3f color;
    HitRecord hitRecord;
    float t = 100000000;
    
    for(int i = 0; i < scene.spheres.size(); i++)
    {
        float temp = IntersectSphere(scene.spheres[i], ray, scene.vertex_data[scene.spheres[i].center_vertex_id-1]);
        if(temp > 0 && temp < t)
        {
            t = temp;
            hitRecord.material = scene.materials[scene.spheres[i].material_id-1];
            hitRecord.position = VectorAddition(ray.origin,ScalarMultiplication(ray.direction,t));
            hitRecord.normal = SphereNormal(hitRecord.position,scene.vertex_data[scene.spheres[i].center_vertex_id-1]);
        }
        
    }

    for(int i = 0; i < scene.triangles.size(); i++)
    {
        if(DotProduct(ray.direction,triangle_normals[i])>0){
            continue;   
        }
        float temp = IntersectTriangles(scene.vertex_data[scene.triangles[i].indices.v0_id-1], scene.vertex_data[scene.triangles[i].indices.v1_id-1], scene.vertex_data[scene.triangles[i].indices.v2_id-1], ray);
        
        if(temp > 0 && temp < t)
        {
            t = temp;
            hitRecord.material = scene.materials[scene.triangles[i].material_id-1];
            hitRecord.position = VectorAddition(ray.origin,ScalarMultiplication(ray.direction,t));
            hitRecord.normal = triangle_normals[i];
        }
        
    }
    
    for(int i = 0; i < scene.meshes.size(); i++)
    {
        for(int j = 0; j < scene.meshes[i].faces.size(); j++)
        {
            if(DotProduct(ray.direction,mesh_face_normals[i][j])>0){
                continue;
            }
            float temp = IntersectTriangles(scene.vertex_data[scene.meshes[i].faces[j].v0_id-1], scene.vertex_data[scene.meshes[i].faces[j].v1_id-1], scene.vertex_data[scene.meshes[i].faces[j].v2_id-1], ray);
            
            if(temp > 0 && temp < t)
            {
                t = temp;
                hitRecord.material = scene.materials[scene.meshes[i].material_id-1];
                hitRecord.position = VectorAddition(ray.origin,ScalarMultiplication(ray.direction,t));
                //printf("plane ray position %.4f %.4f %.4f \n",hitRecord.position.x,hitRecord.position.y,hitRecord.position.z);
                
                hitRecord.normal = mesh_face_normals[i][j];
            }
        }
    }

    if(t < 100000000)
    {
        color = Shading(ray,hitRecord,scene.point_lights,scene.ambient_light);
        return color;
    }
    else if(ray.depth == 0){
        color.x = scene.background_color.x;
        color.y = scene.background_color.y;
        color.z = scene.background_color.z;
        return color;
    }
    else{
        color.x=0;
        color.y=0;
        color.z=0;
        return color;
    }

}

Ray ComputeReflection(Ray ray,Vec3f normal){
    Ray res;
    //ray.direction = Normalize(ray.direction);
    float cross = DotProduct(ScalarMultiplication(ray.direction,-1),normal);
    /*if(cross<0){
        cross=0;
    }*/

    res.direction = VectorAddition(ScalarMultiplication(normal,2*cross),ray.direction);
    res.direction = Normalize(res.direction);
    return res;
} 