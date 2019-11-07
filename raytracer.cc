#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <Eigen/Dense>
#include <bits/stdc++.h> 
#include <stdlib.h>  
#include <sys/stat.h>
#include <unistd.h>
#include <math.h>
#include <iomanip> 
#include <map>
#include <tuple>
#include <tuple>
#include <iterator>
#include <algorithm> 
#include <cmath>


using Eigen::MatrixXd;
using namespace std;
using namespace Eigen;

class Camera{
public:
    Vector3d Eye;
    Vector3d Look;
    Vector3d Up;
    
    Vector3d CUv;
    Vector3d CVv;
    Vector3d CWv;
    
    double cRight, cLeft, cTop, cBottom;
    double dnear;
    
    double width, height;

};

class Sphere {
public:
    Vector3d Cv;    // position
    double radius;
    Vector3d Ka; 
    Vector3d Kd;
    Vector3d Ks;
    Vector3d Kr;
    
    double alpha = 16;
    
//     m index for the sphere's material?
//     
};

// class Material{
// public:
//     Vector3d Ka; 
//     Vector3d Kd;
//     Vector3d Ks;
//     Vector3d Kr;
//     double alpha;
// };

class Light {
public: 
    
    Vector3d p;
    Vector3d e; 
        
};


Vector3d Ambient;
double depthf;


class Ray {
public:
    Vector3d Lv;    // position
    Vector3d Uv;    // direction
    
    void setPosition(Vector3d ptos){
        Lv = ptos;
    }
    
    void setDirection(Vector3d reflection){
        Uv = reflection;
    }
    
};

class Model{
public:

    double wx;
    double wy;
    double wz;
    double theta;
    double scale; 
    double tx;
    double ty;
    double tz;
    string objFilename;
    string mltFilename;
    
};

Ray pixelRay(int& i, int& j, Camera& cam){
    double px, py;
    
    Ray r;
    
    px = (i / (cam.width - 1) * (cam.cRight - cam.cLeft) + cam.cLeft);
    py = (j / (cam.height - 1) * (cam.cBottom - cam.cTop) + cam.cTop);
    
//     cout << cam.cLeft << endl;
    
//     Namming is base point L and direction U

    r.Lv = cam.Eye + (cam.dnear * cam.CWv) + (px * cam.CUv) + (py * cam.CVv);
    
    r.Uv = r.Lv - cam.Eye;
    r.Uv = r.Uv.normalized();
    
    return r;
    
}

struct CollisionReturn {
    bool didCollide;
    double t;
    Vector3d pt;
};
CollisionReturn raySphereTest( Ray& ray, Sphere& sph){
    
    CollisionReturn cr;
//     got r
//     got Cv aka globe
//     got L and U
    
    Vector3d Tv = sph.Cv - ray.Lv;
    double v = Tv.dot(ray.Uv);
    double csq = Tv.dot(Tv);
    
    double disc = (sph.radius * sph.radius ) - ( csq - (v * v));
    
    if (disc < 0){
        cr.didCollide = false;
    }
    else{
        double d = sqrt(disc);
        cr.t = v - d;
        cr.pt = ray.Lv +  cr.t * ray.Uv;
        
        
        cr.didCollide = true;       
    }
    return cr;
}


    
Vector3d raySphereRGB(Ray& ray,  vector<Sphere>& spheres, vector<Light> lights, Vector3d& ambient,int level){
    cout << level << endl << endl;
    
    Vector3d color;
    CollisionReturn hitp;
    
    double best_travel_val = 10000000000;
    double travel_temp;
    Sphere best_sphere;
    Vector3d ptos;
    Vector3d ptosTemp;
    bool bestcollide = false;
    Vector3d tempRGB;
    
    
    for (size_t i = 0; i < spheres.size(); i++ ){
        hitp = raySphereTest(ray, spheres[i]);
        if (hitp.didCollide == true){
           travel_temp = hitp.t;
           ptosTemp = hitp.pt;
           
           
           if (travel_temp > 0 and travel_temp < best_travel_val){
                best_travel_val = travel_temp;
                best_sphere = spheres[i];
                ptos = ptosTemp;
          
                bestcollide = true;

           }

        } 
        
    }      
    
    if (bestcollide == false){
        color << 0 , 0 , 0;
        return color;
    }
                
                Vector3d snrm = ptos - best_sphere.Cv;
                snrm = snrm.normalized();
                
                color = ambient.cwiseProduct(best_sphere.Ka);
                
                for(auto i = lights.begin(); i != lights.end(); i++){
                    
                    Vector3d ptL = i -> p;

                    Vector3d emL = i -> e;
                    
                    Vector3d toL = ptL - ptos;
                    toL = toL.normalized();
                    
//                     shadows: loop through spheres and check if it collides
                    

                    if(snrm.dot(toL) > 0){
                        color += best_sphere.Kd.cwiseProduct(emL) * snrm.dot(toL);
                        
                        Vector3d toC = ray.Lv - ptos;
                        toC = toC.normalized();

                        Vector3d spR = (2 * snrm.dot(toL) * snrm) - toL;
                        spR = spR/spR.norm();//.normalized();
                    
                        double CdR = toC.dot(spR);
                        
                        
//                         KS
                        if (CdR > 0) {
                            color += best_sphere.Ks.cwiseProduct(emL) * pow(CdR, best_sphere.alpha);
                        }
                    }
                    
                }

                if ( level > 0 ) {
//                  do math for reflection ray;
                    Vector3d Uinv = -1 * ray.Uv;
                    Vector3d reflectionRay = (2 * snrm.dot(Uinv) * snrm) - Uinv;
                    reflectionRay = reflectionRay.normalized();

                    
                    Ray refRay;
                    refRay.setPosition(ptos);
                    refRay.setDirection(reflectionRay);
                    color += best_sphere.Kr.cwiseProduct(raySphereRGB(refRay, spheres, lights, Ambient, (level - 1)));
//                     color += best_sphere.Kr * raySphereRGB(refRay, spheres, lights, Ambient, rgbV, (level - 1));
                }
                
                
                
//     compute the vector which is the direction of the reflection ray
// buid a new ray object
//            its position will be ^ ptos

// once we have rr we want to call the recursive function 
    return color;
}




int main (int argc, char *argv[]){
    
    Camera cam;
    Sphere sph;
    Model mod;
//     Material mat;
    Light li;
    
    vector<Light> lights;
    vector<Sphere> spheres;
//     vector<Material> mats;
    string driverFile = argv[1];
    string ppmName = argv[2];
    
    string line;
    size_t sz;


    ifstream inFile(driverFile);
    
    if ( !inFile.is_open()){
    
        cout << "Unable to open";
  
    }
    
    else {
        

        for (int lineNum = 1; getline(inFile, line); lineNum++){
            stringstream ss(line);
            string token;
            
//             cout << line[0] << endl;
            
            if (line[0] == '#'){
                continue;
            }
            
//             cout << "Line #" << lineNum << ":";
            
            for (int tokenNum = 0; ss >> token; tokenNum++){
//                 cout << " " << tokenNum << "." << token;
                
                if (token == "eye"){
                    
                    double dex, dey, dez;
                    
                    ss >> token;
                    dex = stod(token, &sz);
                    
                    ss >> token;
                    dey = stod(token, &sz);
                    
                    ss >> token;
                    dez = stod(token, &sz);
                
                    cam.Eye << dex, dey, dez;
                }
                
                if (token == "look"){
                    
                    double dlx, dly, dlz;
                    
                    ss >> token;
                    dlx = stod(token, &sz);
                    
                    ss >> token;
                    dly = stod(token, &sz);
                    
                    ss >> token;
                    dlz = stod(token, &sz);
                
                    cam.Look << dlx, dly, dlz;
                }
                
                if (token == "up"){
                    
                    double dux, duy, duz;
                    
                    ss >> token;
                    dux = stod(token, &sz);
                    
                    ss >> token;
                    duy = stod(token, &sz);
                    
                    ss >> token;
                    duz = stod(token, &sz);
                
                    cam.Up << dux, duy, duz;
                }
                if (token == "d"){
                    
                    ss >> token;
                    cam.dnear = stod(token, &sz);
                }
                
                 if (token == "bounds"){
                    
                    ss >> token;
                    cam.cLeft = stod(token, &sz);
                     
                    ss >> token;
                    cam.cRight = stod(token, &sz);
                
                    ss >> token;
                    cam.cBottom = stod(token, &sz);
                    
                    ss >> token;
                    cam.cTop = stod(token, &sz);
                }
                
                if (token == "ambient"){
                    
                    double dax, day, daz;
                    
                    ss >> token;
                    dax = stod(token, &sz);
                    
                    ss >> token;
                    day = stod(token, &sz);
                    
                    ss >> token;
                    daz = stod(token, &sz);
                
                    Ambient << dax, day, daz;
                }
                if (token == "res"){
                    
                    
                    ss >> token;
                    cam.width= stod(token, &sz);
                    
                    ss >> token;
                    cam.height = stod(token, &sz);
                    
        
                }
                
                if (token == "light"){
                    
                    
                    double dllx, dlly, dllz;
                    
                    ss >> token;
                    dllx = stod(token, &sz);
                    
                    ss >> token;
                    dlly = stod(token, &sz);
                    
                    ss >> token;
                    dllz = stod(token, &sz);
                
                    li.p << dllx, dlly, dllz;
                    
                    ss >> token;    // skip w token
                    
                    double delx, dely, delz;
                    
                    ss >> token;
                    delx = stod(token, &sz);
                    
                    ss >> token;
                    dely = stod(token, &sz);
                    
                    ss >> token;
                    delz = stod(token, &sz);
                
                    li.e << delx, dely, delz;
                    
                    lights.push_back(li);
                    

                }
                
                 if (token == "sphere"){
                    
                    
                    double dsx, dsy, dsz;
//                     double dr;
                    
                    
                    ss >> token;
                    dsx = stod(token, &sz);
                    
                    ss >> token;
                    dsy = stod(token, &sz);
                    
                    ss >> token;
                    dsz = stod(token, &sz);
                
                    sph.Cv << dsx, dsy, dsz;
                                        
//                  skip w token
                    ss >> token;
                    sph.radius = stod(token, &sz);
                    
                    
                    double dkax, dkay, dkaz;
                    
                    ss >> token;
                    dkax = stod(token, &sz);
                    
                    ss >> token;
                    dkay = stod(token, &sz);
                    
                    ss >> token;
                    dkaz = stod(token, &sz);
                
                    sph.Ka << dkax, dkay, dkaz;
                    
                   
                    
                    double dkdx, dkdy, dkdz;

                    ss >> token;
                    dkdx = stod(token, &sz);
                    
                    ss >> token;
                    dkdy = stod(token, &sz);
                    
                    ss >> token;
                    dkdz = stod(token, &sz);
                
                    sph.Kd << dkdx, dkdy, dkdz;
                    
                    
                    double dksx, dksy, dksz;
                    
                    ss >> token;
                    dksx = stod(token, &sz);
                    
                    ss >> token;
                    dksy = stod(token, &sz);
                    
                    ss >> token;
                    dksz = stod(token, &sz);
                
                    sph.Ks << dksx, dksy, dksz;
                    
                    
                    double dkrx, dkry, dkrz;
                    
                    ss >> token;
                    dkrx = stod(token, &sz);
                    
                    ss >> token;
                    dkry = stod(token, &sz);
                    
                    ss >> token;
                    dkrz = stod(token, &sz);
                
                    sph.Kr << dkrx, dkry, dkrz;
//                     cout << sph.Kr << endl << endl;
                    
                    
                    spheres.push_back(sph);
//                     mats.push_back(mat);
                    
                }
                
                   
                if (token == "model"){
                
                    ss >> token;
                    mod.wx= stod(token, &sz);
                    
                    ss >> token;
                    mod.wy= stod(token, &sz);
                    
                    ss >> token;
                    mod.wz= stod(token, &sz);
                    
                    ss >> token;
                    mod.theta= stod(token, &sz);
                    
                    ss >> token;
                    mod.scale= stod(token, &sz);
                    
                    ss >> token;
                    mod.tx= stod(token, &sz);
                    
                    ss >> token;
                    mod.ty= stod(token, &sz);
                    
                    ss >> token;
                    mod.tz= stod(token, &sz);
                    
                    ss >> token;
                    mod.objFilename= token;
        
                }
                if (token == "recursionlevel"){
                    
                    ss >> token;
                    depthf = stod(token, &sz);
                    
                    
        
                }
             
                
            }

    
        }
        

        
//         Camera Set up
        cam.CWv = cam.Eye - cam.Look;
        cam.CWv = cam.CWv.normalized();
        
        cam.CUv = cam.Up.cross(cam.CWv);
        cam.CUv = cam.CUv.normalized();
        
        cam.CVv = cam.CWv.cross(cam.CUv);
        
        Vector3d rgbV;
        
//         cout << cam.cRight << " " << cam.cLeft << endl;
        //cam.cLeft = -2;
        
        ofstream imgFile(ppmName);
            
            imgFile << "P3" << endl;
            imgFile << cam.width << " " << cam.height << endl;
            imgFile << "255" << endl;
            
//             int depth = 3;
            
            for ( int i = 0; i < cam.height; i++){
                for ( int j = 0; j < cam.width; j++){
                    
                    Ray ray = pixelRay(j, i, cam);

                        rgbV = raySphereRGB(ray, spheres, lights, Ambient, depthf);
                        
                        rgbV(0) = max(0, min(255,int(round(255.0 * rgbV(0)))));
                        rgbV(1) = max(0, min(255,int(round(255.0 * rgbV(1)))));
                        rgbV(2) = max(0, min(255,int(round(255.0 * rgbV(2)))));
                        
                        imgFile << rgbV(0) << " " << rgbV(1) << " " << rgbV(2) << " ";
                 
                }
            }
            


            
            
            
            
            
            imgFile << "/n";
            
        
         
            imgFile.close();
        
        
        

// Model mod;

    cout << depthf << endl;

    cout << "wx: " << mod.wx << endl;
    cout << "wy: " << mod.wy << endl;
    cout << "wz: " << mod.wz << endl;
    cout << "theta: " << mod.theta << endl;
    cout << "scale: " << mod.scale << endl;
    cout << "tx: " << mod.tx << endl;
    cout << "ty: " << mod.ty << endl;
    cout << "tz: " << mod.tz << endl;
    cout << "objFilename: " << mod.objFilename << endl;

        


        
// printing!
        cout << "Eye: " << endl;
        cout << cam.Eye << '\n';
        cout << endl;
        cout << "Look: " << endl;
        cout << cam.Look << endl;
        cout << endl;
        cout << "Up: " << endl;//        handle obj file 

        cout << cam.Up << endl;
        cout << endl;
        cout << "dNear:" << endl;
        cout << cam.dnear << endl;
        cout << "Bounds: " << endl;
        cout << cam.cRight << " " << cam.cLeft << " " << cam.cTop << " " << cam.cBottom << endl;
        cout << endl;
        cout << "Res: " << endl;
        cout << cam.width << endl;
        cout << cam.height << endl;
        
        cout << endl;
        cout << "Ambient:" << endl;
        cout << Ambient << endl;
        
        cout << "radius" << sph.radius << endl;
        

        for(auto i = lights.begin(); i != lights.end(); i++)
        {
            cout << "Lights: " << endl;
            cout << i->p << endl;
            cout << i->e << endl;
        
        }

        for(auto i = spheres.begin(); i != spheres.end(); i++)
        {
            cout << "Spheres: " << endl;
            cout << i->Cv << endl;
            cout << i->radius << endl;
        
        }

      
      
    }
// object transformation
    
    

//     if (!mod.objFilename.empty()){
//        handle obj file 
// 
//         ifstream inFile2(mod.objFilename);
//         
//         if ( !inFile2.is_open()){
//         
//             cout << "Unable to open";
//     
//         }
//         
//         else {
//             
//             
//             
// 
//         
//         
//         
//         }
//     
//     }
    
    
}


