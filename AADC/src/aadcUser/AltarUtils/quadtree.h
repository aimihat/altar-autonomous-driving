//
// Created by robin on 26/07/18.
//

//#include <zconf.h>
#include <cmath>
#include <vector>
#include "../AltarLocalize/stdafx.h"
#include "../AltarUtils/MapStructs.h"
#include <adtf3.h>


class QuadTree{

protected:

public:
    static int total_num_particles;
    static int total_num_nodes;

    //QuadTree(int x1, int y1, int x2, int y2);
    bool divided = false;
    std::vector<Particle> particles;
    int num_particles = 0;
    int min_size = 50;
    std::vector<Particle> basket;

    const int particle_capacity = 4;

    QuadTree* northWest;
    QuadTree* northEast;
    QuadTree* southWest;
    QuadTree* southEast;

    Boundary area;

    QuadTree(Boundary range){

        area = range;
        total_num_nodes += 1;
        northWest = NULL;
        northEast = NULL;
        southWest = NULL;
        southEast = NULL;
    }

    bool Insert(Particle p);
    void Subdivide();
    std::vector<Particle> QueryRange(Boundary range);


};

int QuadTree::total_num_particles = 0;
int QuadTree::total_num_nodes = 0;


bool QuadTree::Insert(Particle p){
    if (!area.ContainsPoint(p.center)){
        return false;
    }

    if (num_particles < particle_capacity){
        particles.push_back(p);
        num_particles += 1;

        total_num_particles++;
        return true;
    }

    if (area.half_distance < min_size){
        particles.push_back(p);
        num_particles += 1;

        total_num_particles++;
        return true;
    }

    if (divided == false){
        Subdivide();

        for(int i = 0; i < num_particles; i++) {

            if (northWest->Insert(particles[i])){
                continue;
            }
            if (northEast->Insert(particles[i])){
                continue;
            }
            if (southWest->Insert(particles[i])){
                continue;
            }
            if (southEast->Insert(particles[i])){
                continue;
            }
        }
    }

    if (northWest->Insert(p)){
        return true;
    }
    if (northEast->Insert(p)){
        return true;
    }
    if (southWest->Insert(p)){
        return true;
    }
    if (southEast->Insert(p)){
        return true;
    }

    return false;
}


void QuadTree::Subdivide(){

    double half_distance = area.half_distance/2;
    double x = area.center.x;
    double y = area.center.x;

    Boundary northWestBoundary;
    northWestBoundary.center.x = x - half_distance;
    northWestBoundary.center.y = y + half_distance;
    northWestBoundary.half_distance = half_distance;

    Boundary northEastBoundary;
    northEastBoundary.center.x = x + half_distance;
    northEastBoundary.center.y = y + half_distance;
    northEastBoundary.half_distance = half_distance;

    Boundary southWestBoundary;
    southWestBoundary.center.x = x - half_distance;
    southWestBoundary.center.y = y - half_distance;
    southWestBoundary.half_distance = half_distance;

    Boundary southEastBoundary;
    southEastBoundary.center.x = x + half_distance;
    southEastBoundary.center.y = y - half_distance;
    southEastBoundary.half_distance = half_distance;

    northWest = new QuadTree(northWestBoundary);
    northEast = new QuadTree(northEastBoundary);
    southWest = new QuadTree(southWestBoundary);
    southEast = new QuadTree(southEastBoundary);

    divided = true;
}


std::vector<Particle> QuadTree::QueryRange(Boundary range){

    if (!area.Intersects(range)){
        return basket;
    }

    if (!divided){

        return particles;

    }

    std::vector<Particle> basket_northWest = northWest->QueryRange(range);
    basket.insert(basket.end(), basket_northWest.begin(),basket_northWest.end());

    std::vector<Particle> basket_northEast = northEast->QueryRange(range);
    basket.insert(basket.end(), basket_northEast.begin(),basket_northEast.end());

    std::vector<Particle> basket_southWest = southWest ->QueryRange(range);
    basket.insert(basket.end(), basket_southWest.begin(),basket_southWest.end());

    std::vector<Particle> basket_southEast = southEast->QueryRange(range);
    basket.insert(basket.end(), basket_southEast.begin(),basket_southEast.end());

    return basket;
}

