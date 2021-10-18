#include <cstdio>
#include <vector>
#include <map>
#include <iostream>
#include "octomap/octomap.h"
#include "octomap/OcTree.h"

// PointKey 构造map的key值
struct PointKey {
    octomap::point3d center;
    PointKey() {}
    PointKey(const octomap::point3d &center) : center(center) {}
    bool operator<(const PointKey &item1) const {
       if(this->center.x() > item1.center.x())
            return true;
        else if(this->center.x() == item1.center.x())
        {
            if(this->center.y() > item1.center.y())
                return true;
            else if(this->center.y() == item1.center.y())
            {
                if(this->center.z() > item1.center.z())
                    return true;
                else
                    return false;
            }
            else
                return false;
        }
        else
            return false;
    }
};

class  APoint {
public:
    APoint();
    ~APoint() = default;
    octomap::point3d center;
    float x;
    float y;
    float z{};
    int depth{};
    int size{};
    double f; //f = g+h
    double g;
    double h;
    APoint *parent;
    // operator  判断是否为中心点
    bool operator==(const APoint &po) const {
        if (center == po.center) {
            return true;
        }
        return false;
    }
};

class CAstar {
    //vector<APoint*> _openList;      //开放列表
    //vector<APoint*> _closeList;     //关闭列表
    std::map<PointKey, APoint *> _openList;
    std::map<PointKey, APoint *> _closeList;
    std::vector<APoint *> _neighbourList;
    APoint *_endPoint;
    APoint *_curPoint;
	int res{2};
public:
    CAstar();
    ~CAstar();
    APoint *findWay(APoint *beginPoint, APoint *endPoint, const octomap::OcTree &tree);
	class APoint *init_pos(const octomap::OcTree &tree, octomap::point3d start);
	void setRes(int resolution) { res = resolution ;}
	int getRes() { return res;}
    //    APoint* findWay(int beginX,int beginY,int endX,int endY);
private:
    static double getF(APoint *point);
    double getH(APoint *point, APoint* curPoint);
    double getG(APoint *point, APoint* curPoint);
	float Distance(octomap::point3d pos1, octomap::point3d pos2);
	int isNeibor(octomap::point3d a, octomap::point3d b,int a_res,int b_res);
	std::vector<APoint *> getNeighboringPoint(APoint *point, const octomap::OcTree &octree);
};
APoint::APoint() : x(0), y(0), h(0), f(0), g(0), parent(nullptr) {
}

CAstar::CAstar() : _endPoint(nullptr), _curPoint(nullptr) {
}

CAstar::~CAstar() {
    _openList.clear();
    _closeList.clear();
    _neighbourList.clear();
}

float CAstar::Distance(octomap::point3d pos1, octomap::point3d pos2) {
    return ((pos1.x() - pos2.x()) * (pos1.x() - pos2.x()) +
            (pos1.y() - pos2.y()) * (pos1.y() - pos2.y()) +
            (pos1.z() - pos2.z()) * (pos1.z() - pos2.z()));
}

int comp(const APoint *s1, const APoint *s2) {
    if (s1->f != s2->f) {
        return s1->f < s2->f;
    } else {
        return false;
    }
}

int CAstar::isNeibor(octomap::point3d a, octomap::point3d b,int a_res,int b_res){   
    int c=0.5*(a_res+b_res); int d=0;
    if(abs(a.x()-b.x())==c){d++;}
    if(abs(a.y()-b.y())==c){d++;}
    if(abs(a.z()-b.z())==c){d++;}
    return (d);
}

APoint *CAstar::findWay(APoint *beginPoint, APoint *endPoint, const octomap::OcTree &tree) {
    _endPoint = endPoint;
    _openList.insert(std::map<PointKey, APoint *>::value_type(PointKey(beginPoint->center), beginPoint));
     std::cout << "navigation!============================================================" << std::endl;
    do {
        //get the curPoint with less f
        auto pit = _openList.begin();
        auto epit = _openList.end();
        auto pit_t = pit;
        pit++;
        for (; pit != epit; pit++) {
            if ((pit->second)->f < (pit_t->second)->f) {
                pit_t = pit;
            }
        }
        APoint *_curPoint = pit_t->second;
        _closeList.insert(std::map<PointKey, APoint *>::value_type(PointKey(_curPoint->center), _curPoint));   
        _openList.erase(pit_t);      //remove from openlist ,put it in closelist
        if (_curPoint->center == _endPoint->center) {      
            std::cout << "have find way" << std::endl;
            return _curPoint;
        }
        std::vector<APoint *> neVec = getNeighboringPoint(_curPoint, tree);
        //cout << "Got _neighbourList" << endl;
        std::map<PointKey, APoint *>::iterator tit;      
        for (auto tmpoint : neVec)
        {              
          if(!(tmpoint==_endPoint))
          {
            PointKey iter(tmpoint->center);     
            tit = _openList.find(iter);      
            if (tit == _openList.end())
            {  //not in openlist , insert it
                tmpoint->parent = _curPoint; 
                tmpoint->g = getG(tmpoint,_curPoint);  
                tmpoint->h = getH(tmpoint,_endPoint);  
                tmpoint->f = getF(tmpoint);                   
                _openList.insert(std::map<PointKey, APoint *>::value_type(PointKey(tmpoint->center), tmpoint)); 
               
            } 
            else 
            { //have been in openlist
                double _g = _curPoint->g +getG(tmpoint,_curPoint);
                if (tmpoint->g > _g)  //determine whether the value of g is smaller than before
                {    
                    tmpoint->parent = _curPoint;
                    tmpoint->g = getG(tmpoint,_curPoint);
                    tmpoint->f = getF(tmpoint);
                    _openList.erase(PointKey(tmpoint->center));     //Update its value
                    _openList.insert(std::map<PointKey, APoint *>::value_type(PointKey(tmpoint->center), tmpoint));
                }
            }
           
           } 
           else      
            {std::cout << "have! find way" << std::endl;
            return tmpoint;}  
            
        }
           

    } while (!_openList.empty());


    std::cout << "---can not find way---" << std::endl;

    return nullptr;
}



std::vector<APoint *> CAstar::getNeighboringPoint(APoint *point, const octomap::OcTree &octree) {
    _neighbourList.clear();
    double lenth = float(0.5) *double(point->size + res);
    int num = 0;
    octomap::point3d min(point->x - lenth, point->y - lenth, point->z - lenth);
    octomap::point3d max(point->x + lenth, point->y + lenth, point->z + lenth);
    std::map<PointKey, APoint *>::iterator ite;   
    for (octomap::OcTree::leaf_bbx_iterator
                 it = octree.begin_leafs_bbx(octree.coordToKey(min), octree.coordToKey(max)),
                 end = octree.end_leafs_bbx();
         it != end; ++it) {
        octomap::OcTreeNode *node = octree.search(it.getCoordinate());
        // cout << " OcuValue = "<<node->getOccupancy()<<" size = "<< it.getSize() <<" depth = "<< it.getDepth() <<endl;
        if (it.getDepth() < 16 && node->getOccupancy() < 0.5 && (!(it.getCoordinate()==point->center))) 
            //find free node with less depth(big voxel)
        {   PointKey itK(it.getCoordinate());     
            ite = _closeList.find(itK);           
            if (ite == _closeList.end()) {
              if(isNeibor(it.getCoordinate(),point->center,it.getSize(),point->size)==1){ //check twice
                 auto *pos = new APoint();
                 pos->center = it.getCoordinate();
                 pos->x = it.getX();
                 pos->y = it.getY();
                 pos->z = it.getZ();
                 pos->depth = it.getDepth();
                 pos->size = it.getSize();
                // pos->h=Distance(it.getCoordinate() , _endPoint->center);
              //  cout << " coor= "<< pos->center <<" depth = "<< pos->depth<<endl;
                 ++num;
               _neighbourList.push_back(pos);
            }
          }
        }
    }
   // cout << "=============neighbour of "<<point->center<<":" << num <<",lenth="<<lenth<< endl;
    return _neighbourList;
}

class APoint *CAstar::init_pos(const octomap::OcTree &tree, octomap::point3d start) {
    octomap::OcTreeNode *cnode = tree.search(start);
	
    if (!cnode || tree.isNodeOccupied(cnode)) {
        return nullptr;
    }
    int num = 0;
    octomap::point3d min(start.x() - res, start.y() - res, start.z() - res);
    octomap::point3d max(start.x() + res, start.y() + res, start.z() + res);
    std::vector<APoint *> poses;
    //cout << "The boundingbox is " << min.x()  << endl;
    for (octomap::OcTree::leaf_bbx_iterator
                 it = tree.begin_leafs_bbx(tree.coordToKey(min), tree.coordToKey(max)),
                 end = tree.end_leafs_bbx();
         it != end; ++it) {
        //cout << "Node center: " << it.getCoordinate() << endl;
        octomap::OcTreeNode *node = tree.search(it.getCoordinate());
        //cout << " OcuValue = "<<node->getOccupancy()<<" size = "<< it.getSize() <<" depth = "<< it.getDepth() <<endl;
        if (it.getDepth() < 16 && node->getOccupancy() < 0.5) //find free node with less depth(big voxel)
        {
            auto *pos = new APoint();
            pos->center = it.getCoordinate();
            pos->x = it.getX();
            pos->y = it.getY();
            pos->z = it.getZ();
            pos->depth = it.getDepth();
            pos->size = it.getSize();
            pos->f = Distance(it.getCoordinate(), start);

            //cout << " coor= " << pos->center << " depth = " << pos->depth << " dis=" << pos->f << endl;
            ++num;
            poses.push_back(pos);
        }
    }
    std::cout << "========init neighbour num= " << num << std::endl;
    sort(poses.begin(), poses.end(), comp);
    return poses[0];
}


double CAstar::getG(APoint *point ,APoint *curPoint) {
    return (curPoint->g + Distance(point->center,curPoint->center));
}

double CAstar::getF(APoint *point) {
    return (point->g + point->h);
}

double CAstar::getH(APoint *point,APoint *endPoint) {
    // return (abs(_endPoint->y - point->y) + abs(_endPoint->x - point->x))*10;
    return Distance(endPoint->center, point->center);
}
