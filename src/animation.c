#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif

#include "geom_utils.h"


#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define NUM_RIGID_BODIES 5
// single body plus screen border body
// for test run
// #define NUM_RIGID_BODIES 3
#define MAX_CONTACT_POINTS NUM_RIGID_BODIES*5 
#define MAX_COLLISIONS_PER_BODY 10
#define BODY_CENTER_TEXTURE_SIZE 10

#define CONTACT_DISTANCE 0.01
// minimum relative velocity to apply impulse
// collision model
#define THRESHOLD 1.0f
// restitution coefficient 
#define EPS 1.0f

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
    Vector2 vrel;
    float vrelNormalProj;
    float penetrationDepth;
    int isColliding;
} PointToEdgeContactPoint;

PointToEdgeContactPoint contactPoints[MAX_CONTACT_POINTS];
int contactPointCount;

RigidBody rigidBodies[NUM_RIGID_BODIES];
RigidBody oldRigidBodies[NUM_RIGID_BODIES];
int iterationCount = 0;

void CopyBodies(RigidBody* destBodies, RigidBody* srcBodies) {
    for (int i=0; i< NUM_RIGID_BODIES; ++i) {
        RigidBody *dst = &destBodies[i];
        RigidBody *src = &srcBodies[i];
        dst->position = src->position;
        dst->angle = src->angle;
        dst->linearVelocity = src->linearVelocity;
        dst->angularVelocity = src->angularVelocity;
    }
}

void resetContactPoints() {
    contactPointCount = 0;
    for(int i=0; i<MAX_CONTACT_POINTS; ++i) {
        contactPoints[i].edgeBody = NULL;
        contactPoints[i].pointBody = NULL;
        contactPoints[i].edgeNormalWorld = (Vector2){0, 0};
        contactPoints[i].contactPointWorld = (Vector2){0, 0};
        // next 2 fields are for debugging only
        contactPoints[i].isColliding = 0;
        contactPoints[i].penetrationDepth = 0;
        contactPoints[i].vrel = (Vector2){INFINITY, INFINITY};
        contactPoints[i].vrelNormalProj = INFINITY;
    }
}

void addContactPoint(PointToEdgeContactPoint cp) {
    contactPoints[contactPointCount] = cp;
    contactPoints[contactPointCount].isColliding = 0;
    contactPoints[contactPointCount].vrel = (Vector2){INFINITY, INFINITY};
    contactPoints[contactPointCount].vrelNormalProj = INFINITY;
    ++contactPointCount;
}

Vector2 transformPointToWorldCoords(
    Vector2 localPoint,
    float angleRad,
    Vector2 position)
{
    Vector2 result = {INFINITY, INFINITY};
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

// compute force and torque at given time t
void ComputeForceAndTorque(
    float time)
{
    // глобальная сила действующая не зависимо от соударений
    Vector2 f = {0, 0};
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        rigidBody->force = f;
        rigidBody->torque = 0;
    }
}

SDL_Texture *rigidBodyTextures[NUM_RIGID_BODIES];
SDL_Texture *bodyCentersTextures[NUM_RIGID_BODIES];

void setupRigidBody(
    RigidBody *rigidBody,
    Vector2 position,
    float mass,
    float angleDegrees,
    float width,
    float height,
    Vector2 linearVelocity,
    float angularVelocityDegPerSec,
    // 1 - if body is a screen border, 0 otherwise
    int isScreenBorder,
    SDL_Renderer *renderer,
    SDL_Texture** drawShapeTexture,
    SDL_Texture** drawShapeCenterTexture
    
    ) {
        rigidBody->isScreenBorder = isScreenBorder;
        rigidBody->position = position;
        rigidBody->angle = M_PI*angleDegrees/180.0f;
        rigidBody->linearVelocity = linearVelocity;
        rigidBody->angularVelocity = 
            M_PI*angularVelocityDegPerSec/180.0f;

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
        0.9*(float)SCREEN_WIDTH,
        0.9*(float)SCREEN_HEIGHT,
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
            1.0e+4,
            // ((float)rand() / RAND_MAX)*5e3 + 1.0e+4, 
            10*(((float)rand() / RAND_MAX)*2 - 1),
            ((float)rand() / RAND_MAX)*100 + 70,
            ((float)rand() / RAND_MAX)*20 + 40,
            (Vector2){30, -30},
            25*(((float)rand() / RAND_MAX)*2 - 1),
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
    int i = 1;
    
    setupRigidBody(
        &rigidBodies[i],
        (Vector2){0.6 * SCREEN_WIDTH, 0.3 * SCREEN_HEIGHT},
        1e+4,
        40,
        100,
        100,
        (Vector2){-10, 0},
        10,
        0,
        renderer,
        &rigidBodyTextures[i],
        &bodyCentersTextures[i]);
    
    ++i;
    
    // smaller body
    setupRigidBody(
        &rigidBodies[i],
        (Vector2){0.3 * SCREEN_WIDTH, 0.3 * SCREEN_HEIGHT},
        1e+4,
        45,
        100,
        100,
        (Vector2){50, 0},
        0,
        0,
        renderer,
        &rigidBodyTextures[i],
        &bodyCentersTextures[i]);

    ++i;

    setupRigidBody(
        &rigidBodies[i],
        (Vector2){0.3 * SCREEN_WIDTH, 0.6 * SCREEN_HEIGHT},
        3e+4,
        45,
        100,
        50,
        (Vector2){50, 0},
        0,
        0,
        renderer,
        &rigidBodyTextures[i],
        &bodyCentersTextures[i]);
    
    ++i;

    setupRigidBody(
        &rigidBodies[i],
        (Vector2){0.6 * SCREEN_WIDTH, 0.6 * SCREEN_HEIGHT},
        2e+4,
        0,
        200,
        50,
        (Vector2){50, 0},
        -10,
        0,
        renderer,
        &rigidBodyTextures[i],
        &bodyCentersTextures[i]);
    
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

void drawColoredSquare(
    SDL_Renderer *renderer,
    int r,
    int g,
    int b,
    int squareSize,
    Vector2 screenPosition)
{

    SDL_SetRenderDrawColor(renderer, r, g, b, 1);
    int rectSize = squareSize;
    SDL_Rect collisionPoint = {
        (int)(screenPosition.x - 0.5 * rectSize),
        (int)(screenPosition.y - 0.5 * rectSize),
        rectSize,
        rectSize};
    SDL_RenderFillRect(renderer, &collisionPoint);
}

void DrawRigidBodies(SDL_Renderer *renderer)
{
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {

        RigidBody *rigidBody = &rigidBodies[i];
        float angle = 180.0f*rigidBody->angle/M_PI;
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
                rigidBody->angle);
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
            drawColoredSquare(
                renderer,
                0, 255, 0,
                10, 
                rigidBody->collisionForces[j].applicationPoint
            );
        }

        // draw contact points 
        for (j =0; j < contactPointCount; ++j) {
            drawColoredSquare(
                renderer,
                0, 255, 255,
                10, 
                contactPoints[j].contactPointWorld
            );
        }
    }
}

void CloseSDL(SDL_Window *window) {
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void getMinMaxPointToEdgeDistance(
    Vector2 pointWorld,
    RigidBody* body,
    // if point is outside the body we found 
    // the minimal distance if point is projected on
    // edge between edge's vertices, otherwise
    // min outside distance is not updated
    float* minOutsideDistance,
    float* minInsideDistance,
    Vector2* closestOutsideNormal,
    Vector2* closestInsideNormal
    
) {
    *minOutsideDistance = INFINITY;
    *minInsideDistance = INFINITY;

    for (int edgeId=0; edgeId<4; ++edgeId) {
        int fistVertexId = edgeId;
        int secondVertexId = (edgeId+1) % 4;
        Vector2 edgeNormal = body->shape.edgeNormals[edgeId];
        edgeNormal = rotateVector(&edgeNormal, body->angle);
        Vector2 edgeFirstVertex = body->shape.endpoints[fistVertexId];
        edgeFirstVertex = transformPointToWorldCoords(
            edgeFirstVertex, body->angle, body->position
        );
        Vector2 edgeSecondVertex = body->shape.endpoints[secondVertexId];
        edgeSecondVertex = transformPointToWorldCoords(
            edgeSecondVertex, body->angle, body->position
        );

        Vector2 pointToLineV;
        // poit penetrartion depth
        float penetrationDepth = pointToLineDistance(
            edgeFirstVertex, edgeSecondVertex,
            pointWorld, &pointToLineV
        );
        float pointLineNormalDot = dotProduct(
            &pointToLineV, &edgeNormal
        );

        if (pointLineNormalDot > 0)
        {
            if (penetrationDepth < *minInsideDistance)
            {
                *minInsideDistance = penetrationDepth;
                *closestInsideNormal = edgeNormal;
            }
        }
        else
        {
            if (penetrationDepth < *minOutsideDistance)
            {
                // check if pointWorld projection to 
                // current edge is between edge's vertices
                Vector2 ptToVert1 = (Vector2) {
                    edgeFirstVertex.x - pointWorld.x,
                    edgeFirstVertex.y - pointWorld.y
                };
                Vector2 ptToVert2 = (Vector2) {
                    edgeSecondVertex.x - pointWorld.x,
                    edgeSecondVertex.y - pointWorld.y
                };
                float ptEdgesDot = dotProduct(
                    &ptToVert1, &ptToVert2
                );
                if (ptEdgesDot < 0) {
                    *minOutsideDistance = penetrationDepth;
                    *closestOutsideNormal = edgeNormal;
                }
            }
        }
    }
}

int IfPointInsideBody(
    Vector2 pointWorld,
    RigidBody* body

) {

    int edgeId = 0;
    float minPenetrationDepth = INFINITY;
    for (edgeId=0; edgeId<4; ++edgeId) {
        int fistVertexId = edgeId;
        int secondVertexId = (edgeId+1) % 4;
        Vector2 edgeNormal = body->shape.edgeNormals[edgeId];
        edgeNormal = rotateVector(&edgeNormal, body->angle);
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
        
    }
    
    return 1;
}

Vector2 bodyPointWorldVelocity(
    RigidBody* body,
    Vector2 bodyPointWorld
) {
    
    Vector3 omega = {0, 0, body->angularVelocity};
    Vector3 r = {
        bodyPointWorld.x - body->position.x,
        bodyPointWorld.y - body->position.y,
        0
    };

    Vector3 vRot = crossProduct(omega, r);
    return (Vector2){
        body->linearVelocity.x + vRot.x,
        body->linearVelocity.y + vRot.y
    };
}

float vrelNormlalProjection(
    RigidBody* body1,
    RigidBody* body2,
    Vector2 b1PointWorld,
    Vector2 b2ClosestNormalWorld,
    Vector2* vectorVrel
) {
    Vector2 v1 = bodyPointWorldVelocity(body1, b1PointWorld);
    Vector2 v2 = bodyPointWorldVelocity(body2, b1PointWorld);
    Vector2 vrel = { v1.x - v2.x, v1.y - v2.y};
    vectorVrel->x = vrel.x;
    vectorVrel->y = vrel.y;
    // relative velocity in b2ClosestNormalWorld direction
    float vRelative = dotProduct(
        &b2ClosestNormalWorld, &vrel);
    return vRelative;
}

float calculateCollisionImpulse(
    float eps,
    RigidBody* body1,
    RigidBody* body2,
    Vector2 b1PointWorld,
    Vector2 b2ClosestNormalWorld
) {
    Vector2 vRelTmp;
    float vrel = vrelNormlalProjection(
        body1, body2,
        b1PointWorld, b2ClosestNormalWorld, &vRelTmp
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

    term3 = dotProduct(
        &b2ClosestNormalWorld,
        &((Vector2){v1tmp.x, v1tmp.y}));

    term4 = dotProduct(
        &b2ClosestNormalWorld,
        &((Vector2){v2tmp.x, v2tmp.y}));

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

void printContact(
    int itCount,
    char* contactTypeText, 
    PointToEdgeContactPoint* cp) {
    printf(
        "%7d: %s depth: %.2e pt: (%.2e %.2e)\n",
        itCount,
        contactTypeText,
        cp->penetrationDepth,
        cp->contactPointWorld.x, cp->contactPointWorld.y
    );
    printf(
        "%7d: vrel (%.2e %.2e) vnp %.2e normal (%.2e %.2e)\n\n",
        itCount,
        cp->vrel.x, cp->vrel.y,
        cp->vrelNormalProj,
        cp->edgeNormalWorld.x, cp->edgeNormalWorld.y
    );
}

int isColliding(PointToEdgeContactPoint *cp)
{
    Vector2 vrel;
    float vNormalProj = vrelNormlalProjection(
        cp->pointBody, cp->edgeBody,
        cp->contactPointWorld, cp->edgeNormalWorld,
        &vrel
    );
    cp->vrelNormalProj = vNormalProj;
    cp->vrel = vrel;

    if (vNormalProj < -THRESHOLD)
    {
        // printContact(iterationCount, "colliding", cp);
        cp->isColliding = 1;
        return 1;
    }
    
    if (fabs(vNormalProj) <= THRESHOLD) {
        // printContact(iterationCount, "resting", cp);
        cp->isColliding = 2;
        return 2;
    }

    if (vNormalProj > THRESHOLD) {
        // printContact(iterationCount, "separating", cp);
        cp->isColliding = 0;
    }
    
    return 0;
}

void updateCollisionImpulses() {

    resetCollisionImpulses();

    int hasCollision = 0;
    // this loop is running until there is
    // no contact point cp, such that isColliding(cp) == 1 left
    do
    {
        hasCollision = 0;

        for (int cId = 0; cId < contactPointCount; ++cId)
        {
            PointToEdgeContactPoint *cp = &contactPoints[cId];
            if (isColliding(cp) == 1)
            {
                hasCollision = 1;
                // calculate pulses for pointBody
                float impulseMag = calculateCollisionImpulse(
                    EPS,
                    cp->pointBody, cp->edgeBody,
                    cp->contactPointWorld, cp->edgeNormalWorld);
                Vector2 momentumPulse = {
                    impulseMag * cp->edgeNormalWorld.x,
                    impulseMag * cp->edgeNormalWorld.y};

                cp->pointBody->collisionMomentumPulse.x = momentumPulse.x;
                cp->pointBody->collisionMomentumPulse.y = momentumPulse.y;

                Vector3 r = {
                    cp->contactPointWorld.x - cp->pointBody->position.x,
                    cp->contactPointWorld.y - cp->pointBody->position.y,
                    0.0};
                Vector3 angMomentumPulse = crossProduct(
                    r,
                    (Vector3){momentumPulse.x, momentumPulse.y, 0.0});

                cp->pointBody->collisionAngleMomentumPulse =
                    angMomentumPulse.z;

                // update 1st body velocities
                cp->pointBody->linearVelocity.x += 
                    cp->pointBody->collisionMomentumPulse.x /
                        cp->pointBody->shape.mass;

                cp->pointBody->linearVelocity.y += 
                    cp->pointBody->collisionMomentumPulse.y /
                        cp->pointBody->shape.mass;

                cp->pointBody->angularVelocity +=
                    cp->pointBody->collisionAngleMomentumPulse /
                    cp->pointBody->shape.momentOfInertia;

                // calculate pulses for edgeBody
                cp->edgeBody->collisionMomentumPulse.x = -momentumPulse.x;
                cp->edgeBody->collisionMomentumPulse.y = -momentumPulse.y;

                r = (Vector3){
                    cp->contactPointWorld.x - cp->edgeBody->position.x,
                    cp->contactPointWorld.y - cp->edgeBody->position.y,
                    0.0};
                angMomentumPulse = crossProduct(
                    r,
                    (Vector3){-momentumPulse.x, -momentumPulse.y, 0.0});
                cp->edgeBody->collisionAngleMomentumPulse = angMomentumPulse.z;


                // update velocities for 2nd body
                cp->edgeBody->linearVelocity.x += 
                    cp->edgeBody->collisionMomentumPulse.x /
                        cp->edgeBody->shape.mass;
                cp->edgeBody->linearVelocity.y += 
                    cp->edgeBody->collisionMomentumPulse.y /
                        cp->edgeBody->shape.mass;

                cp->edgeBody->angularVelocity +=
                    cp->edgeBody->collisionAngleMomentumPulse /
                    cp->edgeBody->shape.momentOfInertia;
            }
        }

    } while (hasCollision == 1);
}


// return values meaning:
// value 0 means no contact
// value 1 means only valid contacts added
// value 2 means simulation recalculation is required

int DetectPointToEdgeContact(
    RigidBody *body1,
    RigidBody *body2)
{

    // screen border can be only used as edges for collision check
    if (body1->isScreenBorder == 1)
    {
        return 0;
    }
    int result = 0;
    int ptId = 0;
    for (ptId = 0; ptId < 4; ++ptId)
    {

        Vector2 currPoint = body1->shape.endpoints[ptId];
        currPoint = transformPointToWorldCoords(
            currPoint, body1->angle, body1->position);
        Vector2 closestNormalWorld = {0, 0};
        int closestEdgeId = -1;
        float penDepth = 0;
        

        Vector2 contactEdgeNormal = {0, 0};
        float minOutDist, minInnerDist;
        Vector2 closestOutNormal = {0, 0};
        Vector2 closestInnnerNormal = {0, 0};
        getMinMaxPointToEdgeDistance(
            currPoint, body2,
            &minOutDist, &minInnerDist,
            &closestOutNormal, &closestInnnerNormal
        );
        // TODO: check case when 
        // body2 is "screen border" body type
        
        int collisionDistanceTest = 0;
        int isInisde = IfPointInsideBody(
            currPoint, body2);
        if (isInisde == 1)
        {
            if (minInnerDist <= CONTACT_DISTANCE) {
                contactEdgeNormal = closestInnnerNormal;
                penDepth = minInnerDist;
                collisionDistanceTest = 1;
                printf(
                    "inner contact depth %.3e < eps %.3e\n",
                    penDepth, CONTACT_DISTANCE);
            }
            else {
                // this means penetration inside is too high
                // we need to recalculate contact points 
                // by restarting simulation at using half 
                // timestep
                if (body2->isScreenBorder == 1) { 
                    // no coll points to be added
                    collisionDistanceTest = 2;
                } else {
                    // printf("bad inner %.3e\n", minInnerDist);
                    return 2;
                }
            }
        } else {
            // this means point is outside 
            if (minOutDist <= CONTACT_DISTANCE) {
                contactEdgeNormal = closestOutNormal;
                penDepth = minOutDist;
                collisionDistanceTest = 1;
                printf(
                    "outer contact depth %.3e < eps %.3e\n",
                    penDepth, CONTACT_DISTANCE
                );
            } else {
                // too far outside is OK (but not for screen border)
                // means no collision point to be added
                if (body2->isScreenBorder) {
                    // printf("bad outer %.3e\n", minOutDist);
                    return 2;
                }
                collisionDistanceTest = 2;
            }
        }

        if (collisionDistanceTest == 1)
        {
            // this means body1 vertex hits body2
            // add new contact point 
            Vector2 normal = contactEdgeNormal;
            if (body2->isScreenBorder)
            {
                normal = (Vector2){
                    -contactEdgeNormal.x,
                    -contactEdgeNormal.y};
            }
            PointToEdgeContactPoint cPoint;
            cPoint.penetrationDepth = penDepth;
            cPoint.contactPointWorld = currPoint;
            cPoint.pointBody = body1;
            cPoint.edgeBody = body2;
            cPoint.edgeNormalWorld = normal;
            addContactPoint(cPoint);
            result = 1;
        }
    }
    return result;
}

// return 1 on if all contact points distance
// is less than tolerance, 0 otherwise
int updateContactPoints() {

    resetContactPoints();
    int checkCount = 0;
    for (int firstId = 0; firstId < NUM_RIGID_BODIES; ++firstId)
    {
        for (int secId = firstId+1; secId < NUM_RIGID_BODIES; ++secId)
        {
            // if (firstId == secId) continue;
            // printf("%d check with %d\n", firstId, secId);
            int testResult = 0;
            testResult = DetectPointToEdgeContact(
                    &rigidBodies[firstId], &rigidBodies[secId]);
            if (testResult == 2) { return 0; }
            if (testResult == 0)
            {
                testResult = DetectPointToEdgeContact(
                    &rigidBodies[secId], &rigidBodies[firstId]);
                if (testResult == 2) { return 0; }
            }
        }
    }
    // if (contactPointCount > 0 ) {
    //     printf("contact point count %d\n", contactPointCount);
    // }
    return 1;
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
    float simulationTime = 0; 
    // dt in seconds
    float dt = 0.01;

    // InitializeRigidBodies(renderer);
    InitializeTestRigidBodies(renderer);
    // test for inner penetration at the start
    // it is supposed we have no bad contact points 
    // at the start
    int cpUpdateResult = 0;
    int quit = 0;
    cpUpdateResult = updateContactPoints();
    if (cpUpdateResult == 0) {
        printf(
            "bad bodies configuration at the start, exiting\n"
        );
        quit = 1;
    }
    SDL_Event e;
    float timeBetweenRender = 1.0/25.0f;
    float frameTime = 0;
    float oldSimTime = simulationTime;
    while (quit == 0)
    {
        
        float smallerDt = dt;
        cpUpdateResult = updateContactPoints();
        int iterCount = 0;
        while (cpUpdateResult == 0)
        {
            // simulate to
            // timestamp = 0.5*(prevSimTime + simulationTime)
            smallerDt *= 0.5;
            // restore old state
            simulationTime = oldSimTime;
            CopyBodies(rigidBodies, oldRigidBodies);
            simulationTime += smallerDt;
            ComputeForceAndTorque(simulationTime);
            RunRigidBodySimulationMidpointV2(smallerDt);
            cpUpdateResult = updateContactPoints();
            ++iterCount;
        }
        updateCollisionImpulses();

        // compute external force , e.g. gravity
        // save state for previous time step
        CopyBodies(oldRigidBodies, rigidBodies);
        oldSimTime = simulationTime;
        ComputeForceAndTorque(simulationTime);
        RunRigidBodySimulationMidpointV2(dt);
        simulationTime += dt;

        ++iterationCount;
        frameTime += dt;

        if (frameTime > timeBetweenRender)
        {
            SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
            SDL_RenderClear(renderer);
            DrawRigidBodies(renderer);
            SDL_RenderPresent(renderer);
            frameTime = 0.0f;
        }

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