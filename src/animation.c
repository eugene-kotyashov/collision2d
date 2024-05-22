#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define NUM_RIGID_BODIES 6
#define MAX_CONTACT_POINTS NUM_RIGID_BODIES*5 
#define MAX_COLLISIONS_PER_BODY 10
#define BODY_CENTER_TEXTURE_SIZE 10
#define COLLISION_FORCE_K 1000.0

//тело должно получить момент импульса при оталкивании
//добавить стокновения 
// колизия записки бара

typedef struct {
    float x;
    float y;
} Vector2;

typedef struct {
    float x;
    float y; 
    float z;
} Vector3;

typedef struct {
    Vector2 force;
    Vector2 applicationPoint;
    // next 2 fields are used in 
    // impulse based collision calculation
    float angularMomentum;
    Vector2 momentum;
} CollisionForce;

typedef struct {
    float width;
    float height;
    float mass;
    float momentOfInertia;
    Vector2 endpoints[4];
    Vector2 edgeNormals[4];

} BoxShape;



typedef struct {
    Vector2 position;
    Vector2 linearVelocity;
    float angle;
    float angularVelocity;
    Vector2 force;
    float torque;
    // силы соударений и точки их приложений
    // заданы в мировой системе координат
    CollisionForce collisionForces[MAX_COLLISIONS_PER_BODY];
    int collisionCount;
    Vector2 collisionMomentumPulse;
    float collisionAngleMomentumPulse;
    int inCollision;
    BoxShape shape;
    int isScreenBorder;
} RigidBody;

typedef struct {
    RigidBody* pointBody;
    RigidBody* edgeBody;
    Vector2 contactPointWorld;
    Vector2 edgeNormalWorld;
} PointToEdgeContactPoint;

PointToEdgeContactPoint contactPoints[MAX_CONTACT_POINTS];
int contactPointCount;

RigidBody rigidBodies[NUM_RIGID_BODIES];

void resetContactPoints() {
    contactPointCount = 0;
    for(int i=0; i<MAX_CONTACT_POINTS; ++i) {
        contactPoints[i].edgeBody = NULL;
        contactPoints[i].pointBody = NULL;
        contactPoints[i].edgeNormalWorld = (Vector2){0, 0};
        contactPoints[i].contactPointWorld = (Vector2){0, 0};
    }
}

void addContactPoint(PointToEdgeContactPoint cp) {
    contactPoints[contactPointCount] = cp;
    ++contactPointCount;
}

float dotProduct(Vector2* v1, Vector2* v2) {
    return v1->x*v2->x + v1->y*v2->y;
}

Vector3 crossProduct(Vector3 w, Vector3 v) {

    return (Vector3){
        w.y*v.z - w.z*v.y,
        w.z*v.x - w.x*v.z,
        w.x*v.y - w.y*v.x
    };
}

float length(Vector2* v) {
    return sqrt(dotProduct(v, v));
}

Vector2 normalize(Vector2* v) {
    float len = length(v);
    Vector2 result;
    result.x = v->x/len;
    result.y = v->y/len;
    return result;
}

float pointToLineDistance(
    Vector2 pt1, Vector2 pt2, Vector2 p, Vector2* pointToLineV) {
    Vector2 lineV = { pt2.x - pt1.x, pt2.y - pt1.y };
    Vector2 lineVnorm = normalize(&lineV);
    Vector2 pointMinusLineStartV = { p.x - pt1.x, p.y - pt1.y};
    float projLen = dotProduct(&pointMinusLineStartV, &lineVnorm);
    Vector2 projV = { projLen*lineVnorm.x, projLen*lineVnorm.y};
    Vector2 ptToLineDistV = { 
        projV.x - pointMinusLineStartV.x,
        projV.y - pointMinusLineStartV.y
    };
    if (pointToLineV != NULL) {
        *pointToLineV = ptToLineDistV;
    }
    return length(&ptToLineDistV);
}

Vector2 rotateVector(Vector2* inV, float angleRad) {
    float cs = cos(angleRad);
    float sc = sin(angleRad);
    Vector2 result = {INFINITY, INFINITY};
    result.x = inV->x*cs - inV->y*sc;
    result.y = inV->x*sc + inV->y*cs;
    return result;
}

void testGeometryFunctions() {
    Vector2 v1 = {1, 1};
    Vector2 v2 = {2, 2};
    float dot = dotProduct(&v1, &v2);
    Vector2 v1norm = normalize(&v1);
    printf("v1:(%e, %e) v2:(%e, %e)\n", v1.x, v1.y, v2.x, v2.y);
    printf("dot %e\n", dot);
    printf("nor v1:(%e, %e)\n", v1norm.x, v1norm.y);
    Vector2 origin = {0, 0};
    Vector2 xAxis = {3, 0};
    Vector2 yAxis = {0, 10};
    Vector2 pt1 = { 1, 1};
    printf(
        "pt1 both dists has to be 1, actial is %e %e\n ",
        pointToLineDistance(origin, xAxis, pt1, NULL),
        pointToLineDistance(origin, yAxis, pt1, NULL)
    );
    Vector2 normal = {1, 1};
    normal = normalize(&normal);
    float len = sqrt(dotProduct(&normal, &normal));
    normal = rotateVector(&normal, 0.25*M_PI);
    float lenRot = sqrt(dotProduct(&normal, &normal));
    printf("len %.2e rotated len %.2e\n", len, lenRot);

    Vector3 i = { 1, 0, 0}; 
    Vector3 j = {0, 1, 0};
    Vector3 k = crossProduct(i, j); 
    printf("cross resul (%.4e %.4e %.4e) expected (0, 0, 1)\n", k.x, k.y, k.z );
}

Vector2 transformPointToWorldCoords(
    Vector2 localPoint,
    float angleDeg,
    Vector2 position)
{
    Vector2 result = {INFINITY, INFINITY};
    float angleRad = M_PI*angleDeg/180;
    //rotation
    result = rotateVector(&localPoint, angleRad);
    // translation
    result.x += position.x;
    result.y += position.y;
    return result;
}

void CalculateBoxInertia(BoxShape *boxShape) {
    float m = boxShape->mass;
    float w = boxShape->width;
    float h = boxShape->height;
    boxShape->momentOfInertia = m * (w * w + h * h) / 12;
}

void ComputeForceAndTorque(RigidBody *rigidBody) {
    // глобальная сила действующая не зависимо от соударений
    Vector2 f = {0, 0};
    rigidBody->force = f;
    rigidBody->torque = 0;
    int i =0;
    for (i=0; i<rigidBody->collisionCount; ++i) {
        Vector2 colForce = rigidBody->collisionForces[i].force;
        rigidBody->force.x += colForce.x;
        rigidBody->force.y += colForce.y;
        // радус вектор верщины участвующей в столкновении
        Vector2 worldEndpoint = 
            rigidBody->collisionForces[i].applicationPoint;
        Vector2 r = {worldEndpoint.x - rigidBody->position.x,
                    worldEndpoint.y - rigidBody->position.y
        };

        float colTorque = r.x*colForce.y - r.y*colForce.x;
        rigidBody->torque += colTorque;
    }
    
}



SDL_Texture *rigidBodyTextures[NUM_RIGID_BODIES];
SDL_Texture *bodyCentersTextures[NUM_RIGID_BODIES];

void resetCollisionForces(RigidBody *body)
{
    Vector2 zeroVec = {0, 0};
    int i = 0;
    body->collisionCount = 0;
    for (i = 0; i < MAX_COLLISIONS_PER_BODY; ++i)
    {
        body->collisionForces[i].force = zeroVec;
        body->collisionForces[i].applicationPoint = zeroVec;
        body->collisionForces[i].angularMomentum = 0;
        body->collisionForces[i].momentum = zeroVec;
    }
}

void setupRigidBody(
    RigidBody *rigidBody,
    Vector2 position,
    float mass,
    float angleDegrees,
    float width,
    float height,
    Vector2 linearVelocity,
    float angularVelocity,
    // 1 - if body is a screen border, 0 otherwise
    int isScreenBorder,
    SDL_Renderer *renderer,
    SDL_Texture** drawShapeTexture,
    SDL_Texture** drawShapeCenterTexture
    
    ) {
        rigidBody->isScreenBorder = isScreenBorder;
        rigidBody->position = position;
        rigidBody->angle = angleDegrees;
        rigidBody->linearVelocity = linearVelocity;
        rigidBody->angularVelocity = angularVelocity;

        BoxShape shape;
        shape.mass = mass;
        shape.width = width;
        shape.height = height;
        shape.endpoints[0].x = -0.5*shape.width, 
        shape.endpoints[0].y = -0.5*shape.height;
        shape.endpoints[1].x = 0.5*shape.width;
        shape.endpoints[1].y = -0.5*shape.height;
        shape.endpoints[2].x =  0.5*shape.width;
        shape.endpoints[2].y = 0.5*shape.height;
        shape.endpoints[3].x = -0.5*shape.width;
        shape.endpoints[3].y = 0.5*shape.height;

        
        shape.edgeNormals[0] = (Vector2){0, -1};
        shape.edgeNormals[1] = (Vector2){1, 0};
        shape.edgeNormals[2] = (Vector2){0, 1};
        shape.edgeNormals[3] = (Vector2){-1, 0};

        resetCollisionForces(rigidBody);

        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;

        // создаем текстуры для тела
        SDL_Surface *surface = SDL_CreateRGBSurface(0, shape.width, shape.height, 32, 0, 0, 0, 0);
        if (isScreenBorder == 1) {
            SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 0, 0, 0));
        } else {
            SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));
        }
        *drawShapeTexture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
        // текстура для центральной точки тела
        surface = SDL_CreateRGBSurface(0, BODY_CENTER_TEXTURE_SIZE, BODY_CENTER_TEXTURE_SIZE, 32, 0, 0, 0, 0);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 0, 0));
        *drawShapeCenterTexture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
}

void InitializeScreenBorder(SDL_Renderer *renderer) {
    // initialize screen's borders rigid body
    setupRigidBody(
        &rigidBodies[0],
        (Vector2)
        {
            0.5*(float)SCREEN_WIDTH,
            0.5*(float)SCREEN_HEIGHT
        },
        INFINITY,
        0,
        (float)SCREEN_WIDTH,
        (float)SCREEN_HEIGHT,
        (Vector2){0, 0},
        0,
        1,
        renderer, 
        &rigidBodyTextures[0],
        &bodyCentersTextures[0]
    );

}


void InitializeRigidBodies(SDL_Renderer *renderer) {

    InitializeScreenBorder(renderer);
    
    for (int i = 1; i < NUM_RIGID_BODIES; ++i) {
        setupRigidBody(
            &rigidBodies[i], 
            (Vector2){rand() % (SCREEN_WIDTH - 50), rand() % (SCREEN_HEIGHT - 50)},
            1, 
            10*(((float)rand() / RAND_MAX)*2 - 1),
            (1 + rand() % 2 )* 100,
            (1 + rand() % 2 )* 30,
            (Vector2){30, -30},
            15*(((float)rand() / RAND_MAX)*2 - 1),
            0,
            renderer,
            &rigidBodyTextures[i],
            &bodyCentersTextures[i]
            );
    }
}

void InitializeTestRigidBodies(SDL_Renderer *renderer)
{
    InitializeScreenBorder(renderer);
    // big body
/*
    setupRigidBody(
        &rigidBodies[1],
        (Vector2){0.5 * SCREEN_WIDTH, 0.5 * SCREEN_HEIGHT},
        1,
        0,
        400,
        50,
        (Vector2){0, 0},
        0,
        0,
        renderer,
        &rigidBodyTextures[0],
        &bodyCentersTextures[0]);
*/
    // smaller body
    setupRigidBody(
        &rigidBodies[1],
        (Vector2){0.2 * SCREEN_WIDTH, 0.3 * SCREEN_HEIGHT},
        1,
        40,
        100,
        40,
        (Vector2){100, -100},
        10,
        0,
        renderer,
        &rigidBodyTextures[1],
        &bodyCentersTextures[1]);
}

SDL_Window *InitSDL() {
    SDL_Window *window = NULL;
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        exit(EXIT_FAILURE);
    }

    window = SDL_CreateWindow("2D Rigid Body Simulation", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN);
    if (window == NULL) {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        SDL_Quit();
        exit(EXIT_FAILURE);
    }

    return window;
}

void DrawRigidBodies(SDL_Renderer *renderer)
{
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {

        RigidBody *rigidBody = &rigidBodies[i];
        float angle = rigidBody->angle;
        SDL_Point bodyCenter = {rigidBody->position.x, rigidBody->position.y};
        SDL_Rect dstrect = {
            (int)(rigidBody->position.x - 0.5 * rigidBody->shape.width),
            (int)(rigidBody->position.y - 0.5 * rigidBody->shape.height),
            (int)(rigidBody->shape.width),
            (int)(rigidBody->shape.height)};

        SDL_RenderCopyEx(
            renderer,
            rigidBodyTextures[i],
            NULL,
            &dstrect,
            angle,
            NULL,
            SDL_FLIP_NONE);

        SDL_Rect centerRect = {
            (int)(rigidBody->position.x - 0.5 * BODY_CENTER_TEXTURE_SIZE),
            (int)(rigidBody->position.y - 0.5 * BODY_CENTER_TEXTURE_SIZE),
            BODY_CENTER_TEXTURE_SIZE,
            BODY_CENTER_TEXTURE_SIZE
        };
        SDL_RenderCopyEx(
            renderer,
            bodyCentersTextures[i],
            NULL,
            &centerRect,
            angle,
            NULL,
            SDL_FLIP_NONE
        );
    
        Vector2 worldEndpoints[4];
        int j = 0;
        for (j = 0; j < 4; ++j) {
        worldEndpoints[j] = transformPointToWorldCoords(
            rigidBody->shape.endpoints[j],
            rigidBody->angle,
            rigidBody->position);
        }
    
    
        for (j = 0; j < 4; ++j)
        {   
            int nextIdx = (j+1) % 4;
            Vector2 p1 = worldEndpoints[j];
            Vector2 p2 = worldEndpoints[nextIdx];
            SDL_SetRenderDrawColor(renderer, 255, 0, 0, 1);
            SDL_RenderDrawLine(renderer, p1.x, p1.y, p2.x, p2.y);
            Vector2 n1 = rotateVector(
                &(rigidBody->shape.edgeNormals[j]),
                M_PI * rigidBody->angle / 180);
            Vector2 n1Start = (Vector2){
                0.5 * (p1.x + p2.x),
                0.5 * (p1.y + p2.y)};
            float normalLength = 20;
            Vector2 n1End = (Vector2){
                n1Start.x + normalLength * n1.x,
                n1Start.y + normalLength * n1.y};
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 1);
            SDL_RenderDrawLine(
                renderer, n1Start.x, n1Start.y, n1End.x, n1End.y
            );
            
        }

        // draw collision forces

        for (j = 0; j < rigidBody->collisionCount; ++j)
        {
            SDL_SetRenderDrawColor(renderer, 0, 255, 0, 1);
            int rectSize = 10;
            SDL_Rect collisionPoint = {
                (int)(rigidBody->collisionForces[j].applicationPoint.x - 0.5 * rectSize),
                (int)(rigidBody->collisionForces[j].applicationPoint.y - 0.5 * rectSize),
                rectSize,
                rectSize};
            SDL_RenderFillRect(renderer, &collisionPoint);
        }
    }
}

void CloseSDL(SDL_Window *window) {
    SDL_DestroyWindow(window);
    SDL_Quit();
}

// this is needed for contact point 
// determination from a "border" object
int IfPointOutsideBody(
    Vector2 pointWorld,
    RigidBody* body,
    Vector2* closestNormalWorld,
    int* closestEdgeId,
    float* penDepth
) {

    int edgeId = 0;
    float minPenetrationDepth = INFINITY;
    for (edgeId=0; edgeId<4; ++edgeId) {
        int fistVertexId = edgeId;
        int secondVertexId = (edgeId+1) % 4;
        Vector2 edgeNormal = body->shape.edgeNormals[edgeId];
        edgeNormal = rotateVector(&edgeNormal, M_PI*body->angle/180.0);
        Vector2 edgeFirstVertex = body->shape.endpoints[fistVertexId];
        edgeFirstVertex = transformPointToWorldCoords(
            edgeFirstVertex, body->angle, body->position
        );
        Vector2 edgeSecondVertex = body->shape.endpoints[secondVertexId];
        edgeSecondVertex = transformPointToWorldCoords(
            edgeSecondVertex, body->angle, body->position
        );

        Vector2 pointToLineV;
        // poit penetrartion deth
        float penetrationDepth = pointToLineDistance(
            edgeFirstVertex, edgeSecondVertex, pointWorld, &pointToLineV
        );
        float pointLineNormalDot = dotProduct(
            &pointToLineV, &edgeNormal
        );

        if (pointLineNormalDot <= 0) {
            // this means point is outside a body
            if (penetrationDepth < minPenetrationDepth) {
                minPenetrationDepth = penetrationDepth;
                *closestEdgeId = edgeId;
                *closestNormalWorld = edgeNormal;
            }
        }

    }
    
    if (minPenetrationDepth == INFINITY) {
        return 0;
    }
    *penDepth = minPenetrationDepth;
    return 1;
}

int IfPointInsideBody(
    Vector2 pointWorld,
    RigidBody* body,
    Vector2* closestNormalWorld,
    int* closestEdgeId,
    float* penDepth
) {

    int edgeId = 0;
    float minPenetrationDepth = INFINITY;
    for (edgeId=0; edgeId<4; ++edgeId) {
        int fistVertexId = edgeId;
        int secondVertexId = (edgeId+1) % 4;
        Vector2 edgeNormal = body->shape.edgeNormals[edgeId];
        edgeNormal = rotateVector(&edgeNormal, M_PI*body->angle/180.0);
        Vector2 edgeFirstVertex = body->shape.endpoints[fistVertexId];
        edgeFirstVertex = transformPointToWorldCoords(
            edgeFirstVertex, body->angle, body->position
        );
        Vector2 edgeSecondVertex = body->shape.endpoints[secondVertexId];
        edgeSecondVertex = transformPointToWorldCoords(
            edgeSecondVertex, body->angle, body->position
        );

        Vector2 pointToLineV;
        // poit penetrartion deth
        float penetrationDepth = pointToLineDistance(
            edgeFirstVertex, edgeSecondVertex, pointWorld, &pointToLineV
        );
        float pointLineNormalDot = dotProduct(
            &pointToLineV, &edgeNormal
        );

        if (pointLineNormalDot <= 0) {
            // this means point is outside a body
            return 0;
        }
        
        if (pointLineNormalDot > 0) {
            if (penetrationDepth < minPenetrationDepth) {
                minPenetrationDepth = penetrationDepth;
                *closestEdgeId = edgeId;
                *closestNormalWorld = edgeNormal;
            }
        }

    }
    *penDepth = minPenetrationDepth;
    return 1;
}

void reflectBodyLinearVelocity(
    RigidBody *rigidBody,
    Vector2 *outerNormal)
{
    float velProj = fabs(dotProduct(
        &(rigidBody->linearVelocity),
        outerNormal));

    rigidBody->linearVelocity.x =
        2.0 * velProj * outerNormal->x + rigidBody->linearVelocity.x;
    rigidBody->linearVelocity.y =
        2.0 * velProj * outerNormal->y + rigidBody->linearVelocity.y;
}

Vector2 bodyPointWorldVelocity(
    RigidBody* body,
    Vector2 bodyPointWorld
) {
    return (Vector2){
        body->linearVelocity.x - 
            body->angularVelocity*(bodyPointWorld.y-body->position.y),
        body->linearVelocity.y +
            body->angularVelocity*(bodyPointWorld.x - body->position.x)
    };
}

float vrelNormlalProjection(
    RigidBody* body1,
    RigidBody* body2,
    Vector2 b1PointWorld,
    Vector2 b2ClosestNormalWorld
) {
    Vector2 v1 = bodyPointWorldVelocity(body1, b1PointWorld);
    Vector2 v2 = bodyPointWorldVelocity(body2, b1PointWorld);
    Vector2 vrel = { v1.x - v2.x, v1.y - v2.y};
    // relative velocity in b2ClosestNormalWorld direction
    float vRelative = dotProduct(
        &b2ClosestNormalWorld, &vrel);
    return vRelative;
}

float calculateCollisionImpulse(
    float eps,
    RigidBody* body1,
    // can be NULL so we use it as a collision with screen border 
    RigidBody* body2,
    Vector2 b1PointWorld,
    Vector2 b2ClosestNormalWorld
) {

    float vrel = vrelNormlalProjection(
        body1, body2, b1PointWorld, b2ClosestNormalWorld
    );
    float numerator = -(1 + eps)*vrel;
    float term1 = 1/ body1->shape.mass;
    float term2 = 1/ body2->shape.mass;
    // vector from body1 center of mass to collision point
    Vector2 rb1 = { 
        b1PointWorld.x - body1->position.x, 
        b1PointWorld.y - body1->position.y
    };
    // the same for body2
    Vector2 rb2 = { 
        b1PointWorld.x - body2->position.x, 
        b1PointWorld.y - body2->position.y
    };
// first body related tmp vector
    Vector3 v1tmp = crossProduct(
        (Vector3){rb1.x, rb1.y, 0.0},
        (Vector3){b2ClosestNormalWorld.x, b2ClosestNormalWorld.y, 0.0}
    ); 
    float invIntertiaB1 = 1/body1->shape.momentOfInertia;
    v1tmp = (Vector3){
        invIntertiaB1 * v1tmp.x,
        invIntertiaB1 * v1tmp.y,
        invIntertiaB1 * v1tmp.z
    };
    v1tmp = crossProduct(
        v1tmp,
        (Vector3) {rb1.x, rb1.y, 0.0}
    );

// second body
    float term3 = 0;
    float term4 = 0;
    if( body2 != NULL) {
        Vector3 v2tmp = crossProduct(
            (Vector3){rb2.x, rb2.y, 0.0},
            (Vector3){b2ClosestNormalWorld.x, b2ClosestNormalWorld.y, 0.0});
        float invIntertiaB2 = 1 / body2->shape.momentOfInertia;
        v2tmp = (Vector3){
            invIntertiaB2 * v2tmp.x,
            invIntertiaB2 * v2tmp.y,
            invIntertiaB2 * v2tmp.z};
        v2tmp = crossProduct(
            v2tmp,
            (Vector3){rb2.x, rb2.y, 0.0});
        float term3 = dotProduct(
            &b2ClosestNormalWorld,
            &((Vector2){v1tmp.x, v1tmp.y}));

        float term4 = dotProduct(
            &b2ClosestNormalWorld,
            &((Vector2){v2tmp.x, v2tmp.y}));
    }
    float j = numerator / (term1 + term2 + term3 + term4);
    return j;
}

void resetCollisionImpulses() {

    for (int i=0; i<NUM_RIGID_BODIES; ++i) {
        rigidBodies[i].collisionMomentumPulse = (Vector2){0, 0};
        rigidBodies[i].collisionAngleMomentumPulse = 0;
        rigidBodies[i].inCollision = 0;
    }
}

void updateCollisionImpulses() {

    resetCollisionImpulses();

    for (int cId = 0; cId < contactPointCount; ++cId)
    {
        float THRESHOLD = 0;
        PointToEdgeContactPoint* cp = &contactPoints[cId];

        // speed of collision point (linear + rotation) of 1st body
        Vector2 v1 = bodyPointWorldVelocity(cp->pointBody, cp->contactPointWorld);
        Vector2 v2 = bodyPointWorldVelocity(cp->edgeBody, cp->contactPointWorld);
        Vector2 vrel = {v1.x - v2.x, v1.y - v2.y};
        // relative velocity in b2ClosestNormalWorld direction
        float vRelative = dotProduct(
            &(cp->edgeNormalWorld), &vrel);

        if (vRelative < -THRESHOLD)
        {
            // apply impulse
            // add collision force based on impusle
            float impulseMag = calculateCollisionImpulse(
                1.0,
                cp->pointBody, cp->edgeBody,
                cp->contactPointWorld, cp->edgeNormalWorld
            );
            Vector2 momentumPulse = {
                impulseMag * cp->edgeNormalWorld.x,
                impulseMag * cp->edgeNormalWorld.y
            };

            cp->pointBody->collisionMomentumPulse =
                momentumPulse;
            Vector3 r = {
                cp->contactPointWorld.x - cp->pointBody->position.x,
                cp->contactPointWorld.y - cp->pointBody->position.y,
                0.0
            };
            Vector3 angMomentumPulse = crossProduct(
                r,
                (Vector3){momentumPulse.x, momentumPulse.y, 0.0}
            );
            cp->pointBody->collisionAngleMomentumPulse =
                angMomentumPulse.z;
            cp->pointBody->inCollision = 1;

            cp->edgeBody->collisionMomentumPulse =
                (Vector2){-momentumPulse.x, -momentumPulse.y};
            r = (Vector3){
                cp->contactPointWorld.x - cp->edgeBody->position.x,
                cp->contactPointWorld.y - cp->edgeBody->position.y,
                0.0};
            angMomentumPulse = crossProduct(
                r,
                (Vector3){-momentumPulse.x, -momentumPulse.y, 0.0});
            cp->edgeBody->collisionAngleMomentumPulse =
                angMomentumPulse.z;
            cp->edgeBody->inCollision = 1;
        } else {
            cp->edgeBody->inCollision = 0;
            cp->pointBody->inCollision = 0;
        }
    }
}

// update forces when edge point from body1
// penetrates body2
void updateCollisionForces(
    RigidBody* body1,
    // can be NULL so we use it as a collision with screen border 
    RigidBody* body2,
    Vector2 b1PointWorld,
    Vector2 b2ClosestNormalWorld,
    int b2ClosestEdgeId,
    float b2PenetrationDepth
) 
{
    
    CollisionForce body1ColForce;
    body1ColForce.applicationPoint = b1PointWorld;
    // используем силу соударения пропроциональную глубине 
    // проникновения
    body1ColForce.force.x = 
        COLLISION_FORCE_K*b2PenetrationDepth*b2ClosestNormalWorld.x;
    body1ColForce.force.y =
        COLLISION_FORCE_K*b2PenetrationDepth*b2ClosestNormalWorld.y;
    if (body1->collisionCount < MAX_COLLISIONS_PER_BODY - 1) {
        //we can now add collision to body1
        body1->collisionForces[body1->collisionCount] = body1ColForce;
        ++(body1->collisionCount);
    }
    if (body2 == NULL) return;
    if (body2->collisionCount < MAX_COLLISIONS_PER_BODY - 1) {
        // add the same force as for 1st body but with opposite sign
        body2->collisionForces[body2->collisionCount] = body1ColForce;
        body2->collisionForces[body2->collisionCount].force.x *= -1.0;
        body2->collisionForces[body2->collisionCount].force.y *= -1.0;
        ++(body2->collisionCount);
    }
}

int DetectPointToEdgeContact(
    RigidBody *body1,
    RigidBody *body2,
    PointToEdgeContactPoint *cPoint)
{

    // screen border can be only used as edges for collision check
    if (body1->isScreenBorder == 1)
    {
        return 0;
    }
    int ptId = 0;
    for (ptId = 0; ptId < 4; ++ptId)
    {

        Vector2 currPoint = body1->shape.endpoints[ptId];
        currPoint = transformPointToWorldCoords(
            currPoint, body1->angle, body1->position);
        Vector2 closestNormalWorld = {0, 0};
        int closestEdgeId = -1;
        float penDepth = 0;
        int testResutl = 0;
        if (body2->isScreenBorder)
        {
            testResutl = IfPointOutsideBody(
                currPoint, body2,
                &closestNormalWorld, &closestEdgeId, &penDepth);
        }
        else
        {
            testResutl = IfPointInsideBody(
                currPoint, body2,
                &closestNormalWorld, &closestEdgeId, &penDepth);
        }
        if (testResutl == 1)
        {
            // reflectBodyLinearVelocity(body1, &closestNormalWorld);
            // this means body1 vertex hits body2
            Vector2 normal = closestNormalWorld;
            if (body2->isScreenBorder)
            {
                normal = (Vector2){
                    -closestNormalWorld.x,
                    -closestNormalWorld.y};
            }
            cPoint->contactPointWorld = currPoint;
            cPoint->pointBody = body1;
            cPoint->edgeBody = body2;
            cPoint->edgeNormalWorld = normal;
            return 1;
        }
    }
    return 0;
}

void updateContactPoints() {

    resetContactPoints();

    for (int firstId = 0; firstId < NUM_RIGID_BODIES; ++firstId)
    {
        for (int secId = 0; secId < NUM_RIGID_BODIES; ++secId)
        {
            if (firstId == secId) continue;
            int testResult = 0;
            PointToEdgeContactPoint newContact;
            if (DetectPointToEdgeContact(
                    &rigidBodies[firstId], &rigidBodies[secId], &newContact) == 0)
            {
                testResult = DetectPointToEdgeContact(
                    &rigidBodies[secId], &rigidBodies[firstId], &newContact);
            } else {
                testResult = 1;
            }
            if (testResult == 1) {
                addContactPoint(newContact);
            }
        }
    }
}

// check collision of body1 verices 
// against body2 edges
// if there is a collision returns 1, 0 otherwise
int DetectBodyToBodyCollisions(
    RigidBody* body1,
    RigidBody* body2 
) {

    // screen border can be only used as edges for collision check
    if (body1->isScreenBorder == 1) { return 0; }
    int ptId = 0;
    for(ptId =0; ptId < 4; ++ptId) {

        Vector2 currPoint = body1->shape.endpoints[ptId];
        currPoint = transformPointToWorldCoords(
            currPoint, body1->angle, body1->position
        );
        Vector2 closestNormalWorld = {0, 0};
        int closestEdgeId = -1;
        float penDepth = 0;
        int testResutl = 0;
        if (body2->isScreenBorder)
        {
            testResutl = IfPointOutsideBody(
                currPoint, body2,
                &closestNormalWorld, &closestEdgeId, &penDepth);
        }
        else
        {
            testResutl = IfPointInsideBody(
                currPoint, body2,
                &closestNormalWorld, &closestEdgeId, &penDepth);
        }
        if (testResutl == 1) {
            // reflectBodyLinearVelocity(body1, &closestNormalWorld);
            // this means body1 vertex hits body2
            Vector2 normal = closestNormalWorld;
            if (body2->isScreenBorder) {
                normal = (Vector2){
                    -closestNormalWorld.x,
                    -closestNormalWorld.y
                };
            }
            updateCollisionForces(body1, body2,
            currPoint, normal, closestEdgeId, penDepth);
            return 1;
        }
    }
    return 0;
}

void DetectCollision(RigidBody *rigidBody) {
    // Столкновение с границами окна
    if (rigidBody->position.x < 0) {
        rigidBody->position.x = 0;
        rigidBody->linearVelocity.x = -rigidBody->linearVelocity.x;
    } else if (rigidBody->position.x + rigidBody->shape.width > SCREEN_WIDTH) {
        rigidBody->position.x = SCREEN_WIDTH - rigidBody->shape.width;
        rigidBody->linearVelocity.x = -rigidBody->linearVelocity.x;
    }

    if (rigidBody->position.y < 0) {
        rigidBody->position.y = 0;
        rigidBody->linearVelocity.y = -rigidBody->linearVelocity.y;
    } else if (rigidBody->position.y + rigidBody->shape.height > SCREEN_HEIGHT) {
        rigidBody->position.y = SCREEN_HEIGHT - rigidBody->shape.height;
        rigidBody->linearVelocity.y = -rigidBody->linearVelocity.y;
    }
}

void DetectColsAndUpdateForces(RigidBody* rigidBodies) {

    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        resetCollisionForces(&rigidBodies[i]);
    }
    

    for (int firstId = 0; firstId < NUM_RIGID_BODIES; ++firstId)
    {
        for (int secId = 0; secId < NUM_RIGID_BODIES; ++secId)
        {
            if (firstId == secId) continue;
            if (DetectBodyToBodyCollisions(
                    &rigidBodies[firstId], &rigidBodies[secId]) == 0)
            {
                DetectBodyToBodyCollisions(
                    &rigidBodies[secId], &rigidBodies[firstId]);
            }
        }
    }
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        ComputeForceAndTorque(&rigidBodies[i]);
    }

}

void RunRigidBodySimulationEuler(SDL_Renderer *renderer, float dt)
{
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        // ПОменять метод Эйлера на метод средней точки
        Vector2 linearAcceleration = (Vector2){
            rigidBody->force.x / rigidBody->shape.mass,
             rigidBody->force.y / rigidBody->shape.mass
        };
        rigidBody->linearVelocity.x += linearAcceleration.x * dt;
        rigidBody->linearVelocity.y += linearAcceleration.y * dt;
        rigidBody->position.x += rigidBody->linearVelocity.x * dt;
        rigidBody->position.y += rigidBody->linearVelocity.y * dt;
        float angularAcceleration = 
            rigidBody->torque / rigidBody->shape.momentOfInertia;
        rigidBody->angularVelocity += angularAcceleration * dt;
        rigidBody->angle += rigidBody->angularVelocity * dt;
    }
    // проверка соударений

    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        resetCollisionForces(&rigidBodies[i]);
    }

    for (int firstId = 0; firstId < NUM_RIGID_BODIES; ++firstId)
    {
        for (int secId = 0; secId < NUM_RIGID_BODIES; ++secId)
        {
            if (firstId == secId) continue;
            if (DetectBodyToBodyCollisions(
                    &rigidBodies[firstId], &rigidBodies[secId]) == 0)
            {
                DetectBodyToBodyCollisions(
                    &rigidBodies[secId], &rigidBodies[firstId]);
            }
        }
    }
    for (int i = 0; i< NUM_RIGID_BODIES; ++i) {
        ComputeForceAndTorque(&rigidBodies[i]);
    }
/*
    for (int i = 0; i< NUM_RIGID_BODIES; ++i) {
        printf(" %i %.4e", i, rigidBodies[i].angle);
    }
    printf("\n");
*/
}

void printAngleStuff() {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        printf(" %i angcel %.4e avel %.4e ang %.4e\n",
               i,
               rigidBodies[i].torque / rigidBodies[i].shape.momentOfInertia,
               rigidBodies[i].angularVelocity,
               rigidBodies[i].angle);
    }
}


// here we use impulses instead of force
// on collision 
// if there is a collision we do't do step on 
// linear and angular velocities we simply set them
// to values based on current collision momentum impulse and 
// andgular momentum pulse value
void RunRigidBodySimulationMidpointV2(
    SDL_Renderer *renderer,
    float *curTime,
    float dt
) {

    updateContactPoints();
    updateCollisionImpulses();
    // if collision detec update linear and angular 
    // velocities
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {

        RigidBody *rigidBody = &rigidBodies[i];
        if (rigidBody->inCollision == 1)
        {
            rigidBody->linearVelocity = (Vector2)
            {
                rigidBody->collisionMomentumPulse.x /
                    rigidBody->shape.mass,
                    
                    rigidBody->collisionMomentumPulse.y /
                    rigidBody->shape.mass
            };

            rigidBody->angularVelocity =
                rigidBody->collisionAngleMomentumPulse /
                rigidBody->shape.momentOfInertia;
        }
    }

    Vector2 stepBeginVelocities[NUM_RIGID_BODIES];
    Vector2 stepBeginPositions[NUM_RIGID_BODIES];
    float stepBeginAngles[NUM_RIGID_BODIES];
    float stepBeginAngularVel[NUM_RIGID_BODIES];

    Vector2 halfStepAccel[NUM_RIGID_BODIES];
    float halfStepAngAccel[NUM_RIGID_BODIES];
    float halfStepAngularVel[NUM_RIGID_BODIES];
    Vector2 halfStepVelocities[NUM_RIGID_BODIES];
    // save step start parameters to use later 
    // for midstep calculation
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody *rigidBody = &rigidBodies[i];
        stepBeginPositions[i] = rigidBody->position;
        stepBeginVelocities[i] = rigidBody->linearVelocity;
        stepBeginAngles[i] = rigidBody->angle;
        stepBeginAngularVel[i] = rigidBody->angularVelocity;
    }

    // do half step calculations
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        
        Vector2 linearAcceleration = (Vector2){
            rigidBody->force.x / rigidBody->shape.mass,
             rigidBody->force.y / rigidBody->shape.mass
        };
        rigidBody->linearVelocity.x += linearAcceleration.x*0.5*dt;
        rigidBody->linearVelocity.y += linearAcceleration.y*0.5*dt;
        rigidBody->position.x += rigidBody->linearVelocity.x*0.5*dt;
        rigidBody->position.y += rigidBody->linearVelocity.y*0.5*dt;
        float angularAcceleration = 
            rigidBody->torque / rigidBody->shape.momentOfInertia;
        rigidBody->angularVelocity += angularAcceleration*0.5*dt;
        rigidBody->angle += rigidBody->angularVelocity*0.5*dt;
        /*
         printf(" %i accel %.4e avel %.4e ang %.4e\n",
             i, angularAcceleration,
              rigidBody->angularVelocity,
              rigidBody->angle );
        */
    }
    
    // init halfstep accelerations and velocities
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        halfStepAccel[i].x = rigidBodies[i].force.x/rigidBodies[i].shape.mass;
        halfStepAccel[i].y = rigidBodies[i].force.y/rigidBodies[i].shape.mass;

        halfStepAngAccel[i] = 
            rigidBodies[i].torque/rigidBodies[i].shape.momentOfInertia;
        halfStepVelocities[i] = rigidBodies[i].linearVelocity;
        halfStepAngularVel[i] = rigidBodies[i].angularVelocity;
    }
    // do fullstep now
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
       
        rigidBody->linearVelocity.x = 
            stepBeginVelocities[i].x + halfStepAccel[i].x * dt;
        rigidBody->linearVelocity.y =
             stepBeginVelocities[i].y + halfStepAccel[i].y * dt;

        rigidBody->position.x = 
            stepBeginPositions[i].x + halfStepVelocities[i].x * dt;
        rigidBody->position.y = 
            stepBeginPositions[i].y + halfStepVelocities[i].y * dt;

        rigidBody->angularVelocity = stepBeginAngularVel[i] + halfStepAngAccel[i] * dt;
        rigidBody->angle = stepBeginAngles[i] + halfStepAngularVel[i] * dt;
    }

    SDL_Delay(dt * 1000);
    *curTime += dt;
    
}



void RunRigidBodySimulationMidpoint(
    SDL_Renderer *renderer,
    float *curTime,
    float dt
) {
    
    Vector2 stepBeginVelocities[NUM_RIGID_BODIES];
    Vector2 stepBeginPositions[NUM_RIGID_BODIES];
    float stepBeginAngles[NUM_RIGID_BODIES];
    float stepBeginAngularVel[NUM_RIGID_BODIES];

    Vector2 halfStepAccel[NUM_RIGID_BODIES];
    float halfStepAngAccel[NUM_RIGID_BODIES];
    float halfStepAngularVel[NUM_RIGID_BODIES];
    Vector2 halfStepVelocities[NUM_RIGID_BODIES];

    // process rigidBodies array starting from index 1
    // because 0 is used for screen borders "body"
    // save step start parameters to use later 
    // for midstep calculation
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody *rigidBody = &rigidBodies[i];
        stepBeginPositions[i] = rigidBody->position;
        stepBeginVelocities[i] = rigidBody->linearVelocity;
        stepBeginAngles[i] = rigidBody->angle;
        stepBeginAngularVel[i] = rigidBody->angularVelocity;
    }

    // проверка соударений
    // вычисление сил и моментов сил и скоростей
    DetectColsAndUpdateForces(rigidBodies);
    
    // do half step calculations
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        
        Vector2 linearAcceleration = {0, 0};
        /*
        float angularAcceleration = 0;
        if (rigidBody->shape.mass != 0)
        {
        */
            linearAcceleration = (Vector2){
                rigidBody->force.x / rigidBody->shape.mass,
                rigidBody->force.y / rigidBody->shape.mass
            };
        // }
        rigidBody->linearVelocity.x += linearAcceleration.x*0.5*dt;
        rigidBody->linearVelocity.y += linearAcceleration.y*0.5*dt;
        rigidBody->position.x += rigidBody->linearVelocity.x*0.5*dt;
        rigidBody->position.y += rigidBody->linearVelocity.y*0.5*dt;

        float angularAcceleration = 
            rigidBody->torque / rigidBody->shape.momentOfInertia;
        rigidBody->angularVelocity += angularAcceleration*0.5*dt;
        rigidBody->angle += rigidBody->angularVelocity*0.5*dt;
        /*
         printf(" %i accel %.4e avel %.4e ang %.4e\n",
             i, angularAcceleration,
              rigidBody->angularVelocity,
              rigidBody->angle );
        */
    }
    // проверка соударений
    // вычисление сил и моментов сил и скоростей
    DetectColsAndUpdateForces(rigidBodies);
    // init halfstep accelerations and velocities
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        halfStepAccel[i].x = rigidBodies[i].force.x/rigidBodies[i].shape.mass;
        halfStepAccel[i].y = rigidBodies[i].force.y/rigidBodies[i].shape.mass;

        halfStepAngAccel[i] = 
            rigidBodies[i].torque/rigidBodies[i].shape.momentOfInertia;
        halfStepVelocities[i] = rigidBodies[i].linearVelocity;
        halfStepAngularVel[i] = rigidBodies[i].angularVelocity;
    }
    // do fullstep now
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
       
        rigidBody->linearVelocity.x = 
            stepBeginVelocities[i].x + halfStepAccel[i].x * dt;
        rigidBody->linearVelocity.y =
             stepBeginVelocities[i].y + halfStepAccel[i].y * dt;

        rigidBody->position.x = 
            stepBeginPositions[i].x + halfStepVelocities[i].x * dt;
        rigidBody->position.y = 
            stepBeginPositions[i].y + halfStepVelocities[i].y * dt;

        rigidBody->angularVelocity = stepBeginAngularVel[i] + halfStepAngAccel[i] * dt;
        rigidBody->angle = stepBeginAngles[i] + halfStepAngularVel[i] * dt;
    }

    SDL_Delay(dt * 1000);
    *curTime += dt;
    
}

int main() {
    SDL_Window *window = InitSDL();
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (renderer == NULL) {
        printf("Renderer could not be created! SDL_Error: %s\n", SDL_GetError());
        CloseSDL(window);
        exit(EXIT_FAILURE);
    }

    testGeometryFunctions();

    float totalSimulationTime = 10000; 
    float currentTime = 0; 
    float dt = 1.0 / 60.0; 

    InitializeRigidBodies(renderer);
    // InitializeTestRigidBodies(renderer);

    SDL_Event e;
    int quit = 0;
    while (quit == 0)
    {
        if (currentTime < totalSimulationTime)
        {
            RunRigidBodySimulationMidpoint(
                renderer, &currentTime, dt);
            // RunRigidBodySimulationEuler(renderer, dt);
        }

        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255); 
        SDL_RenderClear(renderer);

        DrawRigidBodies(renderer);

        SDL_RenderPresent(renderer);

        while (SDL_PollEvent(&e))
        {
            if (e.type == SDL_QUIT)
            {
                quit = 1;
            }
            if (e.type == SDL_KEYDOWN)
            {
                quit = 1;
            }
            if (e.type == SDL_MOUSEBUTTONDOWN)
            {
                quit = 1;
            }
        }
    }
    
    
    CloseSDL(window);
    return 0;
}