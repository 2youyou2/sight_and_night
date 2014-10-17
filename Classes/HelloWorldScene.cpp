#include "HelloWorldScene.h"
#include "Triangulate.h"

USING_NS_CC;

Scene* HelloWorld::createScene()
{
    // 'scene' is an autorelease object
    auto scene = Scene::create();
    
    // 'layer' is an autorelease object
    auto layer = HelloWorld::create();

    // add layer as a child to scene
    scene->addChild(layer);

    // return the scene
    return scene;
}



struct Intersect
{
    float x;
    float y;
    float param;
    float angle;
};

struct Segment
{
    Vec2 a;
    Vec2 b;
};

struct Polygon : std::vector<Intersect>
{
    Color4F color;
};
std::vector<Segment> segments;
Vec2 mouse;
bool mouseMoved = true;

DrawNode* stencil;
DrawNode* ctx;

void addSegments(Vec2* segs, int count)
{
    for (int i=0; i<count; i++)
    {
        Segment s;
        s.a = segs[i];

        if(i == count-1)
            s.b = segs[0];
        else
            s.b = segs[i+1];

        segments.push_back(s);
    }
}

void initSegments()
{
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();
    
    float l = origin.x;
    float b = origin.y;
    float r = origin.x + visibleSize.width;
    float t = origin.y + visibleSize.height;
    
    // border
    Vec2 segs[4] = {Vec2(l,b), Vec2(r,b), Vec2(r,t), Vec2(l,t)};
    addSegments(segs, 4);
    
    // POLYGON #1
    Vec2 p1[4] = {Vec2(100, 150), Vec2(120, 50), Vec2(200, 80), Vec2(140, 210)};
    addSegments(p1, 4);

    // POLYGON #2
    Vec2 p2[4] = {Vec2(100, 300), Vec2(120, 450), Vec2(60, 400), Vec2(100, 300)};
    addSegments(p2, 4);

    // POLYGON #3
    Vec2 p3[4] = {Vec2(250, 260), Vec2(270, 150), Vec2(350, 200), Vec2(400, 320)};
    addSegments(p3, 4);
    
    // POLYGON #4
    Vec2 p4[3] = {Vec2(540, 60), Vec2(560, 40), Vec2(570, 70)};
    addSegments(p4, 3);
    
    // POLYGON #5
    Vec2 p5[4] = {Vec2(650, 390), Vec2(760, 370), Vec2(740, 470), Vec2(630, 490)};
    addSegments(p5, 4);
    
    // POLYGON #6
    Vec2 p6[3] = {Vec2(600, 195), Vec2(780, 150), Vec2(680, 250)};
    addSegments(p6, 3);
}


// Find intersection of RAY & SEGMENT
bool getIntersection(const Segment& ray, const Segment& segment, Intersect& out){
    
	// RAY in parametric: Point + Delta*T1
	float r_px = ray.a.x;
	float r_py = ray.a.y;
	float r_dx = ray.b.x-ray.a.x;
	float r_dy = ray.b.y-ray.a.y;
    
	// SEGMENT in parametric: Point + Delta*T2
	float s_px = segment.a.x;
	float s_py = segment.a.y;
	float s_dx = segment.b.x-segment.a.x;
	float s_dy = segment.b.y-segment.a.y;
    
	// Are they parallel? If so, no intersect
	float r_mag = sqrt(r_dx*r_dx+r_dy*r_dy);
	float s_mag = sqrt(s_dx*s_dx+s_dy*s_dy);
	if(r_dx/r_mag==s_dx/s_mag && r_dy/r_mag==s_dy/s_mag){
		// Unit vectors are the same.
		return false;
	}
    
	// SOLVE FOR T1 & T2
	// r_px+r_dx*T1 = s_px+s_dx*T2 && r_py+r_dy*T1 = s_py+s_dy*T2
	// ==> T1 = (s_px+s_dx*T2-r_px)/r_dx = (s_py+s_dy*T2-r_py)/r_dy
	// ==> s_px*r_dy + s_dx*T2*r_dy - r_px*r_dy = s_py*r_dx + s_dy*T2*r_dx - r_py*r_dx
	// ==> T2 = (r_dx*(s_py-r_py) + r_dy*(r_px-s_px))/(s_dx*r_dy - s_dy*r_dx)
	float T2 = (r_dx*(s_py-r_py) + r_dy*(r_px-s_px))/(s_dx*r_dy - s_dy*r_dx);
	float T1 = (s_px+s_dx*T2-r_px)/r_dx;
    
	// Must be within parametic whatevers for RAY/SEGMENT
	if(T1<0) return false;
	if(T2<0 || T2>1) return false;
    
	// Return the POINT OF INTERSECTION
	out = { r_px+r_dx*T1, r_py+r_dy*T1, T1 };
    return true;
}

bool compare_angle(const Intersect& t1, const Intersect& t2)
{
    return t1.angle < t2.angle;
}

Polygon getSightPolygon(float sightX, float sightY)
{
    
	// Get all unique points
    std::vector<Vec2> points;
    for (Segment& s : segments)
    {
        bool findA = false;
        bool findB = false;
        
        for(Vec2& p : points)
        {
            if(p.x == s.a.x && p.y == s.a.y)
                findA = true;
            if(p.x == s.b.x && p.y == s.b.y)
                findB = true;
        }
        
        if(!findA)
            points.push_back(s.a);
        if(!findB)
            points.push_back(s.b);
    }
    
	// Get all angles
    std::vector<float> uniqueAngles;
	for(int j=0;j<points.size();j++){
		Vec2 uniquePoint = points[j];
		float angle = atan2(uniquePoint.y-sightY,uniquePoint.x-sightX);

		uniqueAngles.push_back(angle-0.0001);
        uniqueAngles.push_back(angle);
        uniqueAngles.push_back(angle+0.0001);
	}
    
	// RAYS IN ALL DIRECTIONS
    Polygon intersects;
	for(int j=0;j<uniqueAngles.size();j++){
		float angle = uniqueAngles[j];
        
		// Calculate dx & dy from angle
		float dx = cos(angle);
		float dy = sin(angle);
        
		// Ray from center of screen to mouse
		Segment ray = {
            Vec2(sightX, sightY),
            Vec2(sightX+dx, sightY+dy)
		};
        
		// Find CLOSEST intersection
        bool first = true;
		Intersect closestIntersect;
		for(int i=0;i<segments.size();i++){
			Intersect intersect;
            if(!getIntersection(ray,segments[i], intersect))
                continue;
			if(first || intersect.param<closestIntersect.param){
				closestIntersect=intersect;
                first = false;
			}
		}
        
		// Intersect angle
		if(first) continue;
		closestIntersect.angle = angle;
        
		// Add to list of intersects
		intersects.push_back(closestIntersect);
        
	}
    
    sort(intersects.begin(), intersects.end(), compare_angle);
    
	// Polygon is intersects, in order of angle
	return intersects;
    
}


// on "init" you need to initialize your instance
bool HelloWorld::init()
{
    //////////////////////////////
    // 1. super init first
    if ( !Layer::init() )
    {
        return false;
    }
    
    initSegments();
    
    Size visibleSize = Director::getInstance()->getVisibleSize();
    Vec2 origin = Director::getInstance()->getVisibleOrigin();

    mouse = Vec2(visibleSize.width/2 + origin.x, visibleSize.height/2 + origin.y);
    
    auto back = Sprite::create("background.png");
    back->setPosition(mouse);
    addChild(back);
    
    
    Size size = back->getContentSize();
    float radio = visibleSize.width/size.width;
    back->setScale(radio);
    
    ClippingNode* clipper = ClippingNode::create();
    clipper->setDrawStencil(true);
    this->addChild(clipper);
    
    stencil = DrawNode::create();
    clipper->setStencil(stencil);
    
    ctx = DrawNode::create();
    this->addChild(ctx);

    auto fore = Sprite::create("foreground.png");
    fore->setPosition(Vec2(visibleSize.width/2 + origin.x, visibleSize.height/2 + origin.y));
    BlendFunc func = {GL_DST_COLOR, GL_NONE};
    fore->setBlendFunc(func);
    clipper->addChild(fore);
    
    fore->setScale(radio);
    
    auto listener = EventListenerTouchOneByOne::create();
    listener->setSwallowTouches(true);
    
    listener->onTouchBegan = [](Touch* touch, Event* event){
        mouse = touch->getLocation();
        mouseMoved = true;
        return true;
    };
    listener->onTouchEnded = [](Touch* touch, Event* event){};
    
    listener->onTouchMoved = [](Touch* touch, Event* event){
        mouse = touch->getLocation();
        mouseMoved = true;
    };

    
    EventDispatcher* dispatcher = Director::getInstance()->getEventDispatcher();
    dispatcher->addEventListenerWithSceneGraphPriority(listener, this);
    
    this->scheduleUpdate();
    
    return true;
}

void drawPolygon(Polygon& polygon, Color4F fillStyle)
{
    if(polygon.size() == 0)
        return;

    Vector2dVector vecs;
    for(Intersect& i : polygon)
    {
        vecs.push_back(Vector2d(i.x, i.y));
//        ctx->drawDot(Vec2(i.x, i.y), 2, Color4F(1,1,1,1));
    }
    
    Vector2dVector result;
    Triangulate::Process(vecs, result);
    
    int tcount = result.size()/3;
    for (int i = 0; i<tcount; i++)
    {
        Vec2 v[3];
        v[0] = Vec2(result[i*3+0].GetX(), result[i*3+0].GetY());
        v[1] = Vec2(result[i*3+1].GetX(), result[i*3+1].GetY());
        v[2] = Vec2(result[i*3+2].GetX(), result[i*3+2].GetY());
        stencil->drawTriangle(v[0], v[1], v[2], fillStyle);
        
//        stencil->drawPolygon(v, 3, fillStyle);
    }
    
}

void HelloWorld::update(float dt)
{
    if(!mouseMoved)
        return;
    
    stencil->clear();
    ctx->clear();
    
	// Sight Polygons
	float fuzzyRadius = 10;
	std::vector<Polygon> polygons;
    polygons.push_back(getSightPolygon(mouse.x, mouse.y));
    
	for(float angle=0; angle<M_PI*2; angle+=(M_PI*2)/10)
    {
		float dx = cos(angle)*fuzzyRadius;
		float dy = sin(angle)*fuzzyRadius;
		polygons.push_back(getSightPolygon(mouse.x+dx, mouse.y+dy));
	};
    
    Color4F white(1,1,1,1);
    Color4F transparentWhite(1,1,1,0.2f);
    
	// DRAW AS A GIANT POLYGON
	for(int i=1; i<polygons.size(); i++)
    {
		drawPolygon(polygons[i], transparentWhite);
	}
    drawPolygon(polygons[0], white);
    
	// Draw dots
    ctx->drawDot(mouse, 2, white);
	for(float angle=0; angle<M_PI*2; angle+=(M_PI*2)/10)
    {
		float dx = cos(angle)*fuzzyRadius;
		float dy = sin(angle)*fuzzyRadius;
        ctx->drawDot(mouse+Vec2(dx,dy), 2, white);
    }
    
    for(int i=0; i<segments.size(); i++)
    {
		Segment& seg = segments[i];
        ctx->drawSegment(seg.a, seg.b, 1, white);
	}
    
    mouseMoved = false;
}


void HelloWorld::draw(Renderer *renderer, const Mat4 &transform, uint32_t flags)
{
}
