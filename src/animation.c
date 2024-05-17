#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define NUM_RIGID_BODIES 2
#define BODY_CENTER_TEXTURE_SIZE 10
#define COLLISION_FORCE_VALUE 10000
//тело должно получить момент импульса при оталкивании
//добавить стокновения 
// колизия записки бара

typedef struct {
    float x;
    float y;
} Vector2;

typedef struct {
    float width;
    float height;
    float mass;
    float momentOfInertia;
    Vector2 endpoints[4];
    Vector2 edgeNormals[4];
    // силы соударений действующие на каждую верщину 
    // заданы в мировой системе координат
    Vector2 collisionEndpointForces[4];
} BoxShape;



typedef struct {
    Vector2 position;
    Vector2 linearVelocity;
    float angle;
    float angularVelocity;
    Vector2 force;
    float torque;
    BoxShape shape;
} RigidBody;

RigidBody rigidBodies[NUM_RIGID_BODIES];

float dotProduct(Vector2* v1, Vector2* v2) {
    return v1->x*v2->x + v1->y*v2->y;
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
        pointMinusLineStartV.x - projV.x,
        pointMinusLineStartV.y - projV.y
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
    printf("len %.2e ratated len %.2e", len, lenRot);
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
    for (i=0; i<4; ++i) {
        Vector2 endpointForce = rigidBody->shape.collisionEndpointForces[i];
        rigidBody->force.x += endpointForce.x;
        rigidBody->force.y += endpointForce.y;
        // радус вектор верщины участвующей в столкновении
        Vector2 worldEndpoint = transformPointToWorldCoords(
            rigidBody->shape.endpoints[i],
            rigidBody->angle,
            rigidBody->position );
        Vector2 r = {worldEndpoint.x - rigidBody->position.x,
                    worldEndpoint.y - rigidBody->position.y
        };

        float endpointTorque = r.x*endpointForce.y - r.y*endpointForce.x;
        rigidBody->torque += endpointTorque;
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
    float angularVelocity,
    SDL_Renderer *renderer,
    SDL_Texture** drawShapeTexture,
    SDL_Texture** drawShapeCenterTexture
    
    ) {
    
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

        // edge endpoints (start, end) : 
        // egde 0 (0, 1), edge 1 (1, 2), edge 2 (2, 3), edge 4 (3, 1)
        shape.edgeNormals[0] = (Vector2){0, -1};
        shape.edgeNormals[1] = (Vector2){1, 0};
        shape.edgeNormals[2] = (Vector2){0, 1};
        shape.edgeNormals[3] = (Vector2){-1, 0};

        Vector2 zeroVec = {0, 0};
        shape.collisionEndpointForces[0] = zeroVec;
        shape.collisionEndpointForces[1] = zeroVec;
        shape.collisionEndpointForces[2] = zeroVec;
        shape.collisionEndpointForces[3] = zeroVec;

        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;

        // создаем текстуры для тела
        SDL_Surface *surface = SDL_CreateRGBSurface(0, shape.width, shape.height, 32, 0, 0, 0, 0);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));
        *drawShapeTexture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
        // текстура для центральной точки тела
        surface = SDL_CreateRGBSurface(0, BODY_CENTER_TEXTURE_SIZE, BODY_CENTER_TEXTURE_SIZE, 32, 0, 0, 0, 0);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 0, 0));
        *drawShapeCenterTexture = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
}



void InitializeRigidBodies(SDL_Renderer *renderer) {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        setupRigidBody(
            &rigidBodies[i], 
            (Vector2){rand() % (SCREEN_WIDTH - 50), rand() % (SCREEN_HEIGHT - 50)},
            1, 
            (rand() % 360) / 360.f * M_PI * 2,
            (1 + rand() % 2 )* 100,
            (1 + rand() % 2 )* 30,
            (Vector2){100, -100},
            10,
            renderer,
            &rigidBodyTextures[i],
            &bodyCentersTextures[i]
            );
    }
}

void InitializeTestRigidBodies(SDL_Renderer *renderer)
{

    setupRigidBody(
        &rigidBodies[0],
        (Vector2){0.5 * SCREEN_WIDTH, 0.5 * SCREEN_HEIGHT},
        1,
        0,
        200,
        50,
        (Vector2){0, 0},
        0,
        renderer,
        &rigidBodyTextures[0],
        &bodyCentersTextures[0]);

    setupRigidBody(
        &rigidBodies[1],
        (Vector2){0.2 * SCREEN_WIDTH, 0.1 * SCREEN_HEIGHT},
        1,
        0,
        50,
        20,
        (Vector2){100, -100},
        10,
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
            (int)(rigidBody->shape.height)
        };
        SDL_RenderCopyEx(
            renderer,
            rigidBodyTextures[i],
            NULL,
            &dstrect,
            angle,
            NULL,
            SDL_FLIP_NONE
        );

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

    }
}

void CloseSDL(SDL_Window *window) {
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void DetectLineSegmentCollision(
    Vector2 *lineStart,
    Vector2 *lineEnd,
    // normal vector pointing outside of the object
    Vector2 *outerNormal,
    RigidBody *rigidBody) 
{
    int i;
    for (i=0; i<4; ++i) {
        Vector2 bodyPointWorld = transformPointToWorldCoords(
            rigidBody->shape.endpoints[i],
            rigidBody->angle,
            rigidBody->position
        );
        Vector2 pointToLineV;
        float distance = pointToLineDistance(
            *lineStart, *lineEnd, bodyPointWorld, &pointToLineV
        );
        float pointLineNormalDot = dotProduct(
            &pointToLineV, outerNormal
        );

        if (pointLineNormalDot < 0)
        {
            // check if projection of endpoint
            // onto line segment is
            // between  line endpoints
            Vector2 pointToLineProjectionPoint = bodyPointWorld;
            pointToLineProjectionPoint.x += pointToLineV.x;
            pointToLineProjectionPoint.y += pointToLineV.y;
            Vector2 lineVector1 = {
                pointToLineProjectionPoint.x - lineStart->x,
                pointToLineProjectionPoint.y - lineStart->y};
            Vector2 lineVector2 = {
                pointToLineProjectionPoint.x - lineEnd->x,
                pointToLineProjectionPoint.y - lineEnd->y};

            float dot = dotProduct(&lineVector1, &lineVector2);
            if (dot < 0)
            {
                
                // adding collision force
                /*
                rigidBody->shape.collisionEndpointForces[i].x =
                    COLLISION_FORCE_VALUE * outerNormal->x;
                rigidBody->shape.collisionEndpointForces[i].y =
                    COLLISION_FORCE_VALUE * outerNormal->y;
                */
            
               rigidBody->angularVelocity = -rigidBody->angularVelocity;
               float velProj = fabs(dotProduct(&rigidBody->linearVelocity, outerNormal));
               rigidBody->linearVelocity.x = 2.0*velProj*outerNormal->x +
                    rigidBody->linearVelocity.x;
               rigidBody->linearVelocity.y = 2.0*velProj*outerNormal->y +
                    rigidBody->linearVelocity.y ;
                
            }
            else
            {
                rigidBody->shape.collisionEndpointForces[i].x =
                    0;
                rigidBody->shape.collisionEndpointForces[i].y =
                    0;
            }

            break;
        }
        /*
        if (i == 0) {
            printf(
                " %d: %.2e %.2e (%.2e, %.2e)",
                i, distance, pointLineNormalDot,
                rigidBody->shape.collisionEndpointForces[i].x,
                rigidBody->shape.collisionEndpointForces[i].y
            );
        }
        */
    }

   //  printf("\n");

}

void DetectCollisionNew(RigidBody *rigidBody) {
    // detect collision with screen borders

    // screen bottom
    Vector2 screenBottom[2] = { 
        {0, SCREEN_HEIGHT}, {SCREEN_WIDTH, SCREEN_HEIGHT}
    };
    Vector2 screenBottomNormal = {0, -1};

    DetectLineSegmentCollision(
        &screenBottom[0],
        &screenBottom[1],
        &screenBottomNormal,
        rigidBody
    );
    // screen top
    Vector2 screenTop[2] = { 
        {0, 0}, {SCREEN_WIDTH, 0}
    };
    Vector2 screenTopNormal = {0, 1};

    DetectLineSegmentCollision(
        &screenTop[0],
        &screenTop[1],
        &screenTopNormal,
        rigidBody
    );

    // screen left
    Vector2 screenLeft[2] = { 
        {0, 0}, {0, SCREEN_HEIGHT}
    };
    Vector2 screenLeftNormal = {1, 0};

    DetectLineSegmentCollision(
        &screenLeft[0],
        &screenLeft[1],
        &screenLeftNormal,
        rigidBody
    );

    Vector2 screenRight[2] = { 
        {SCREEN_WIDTH, 0}, {SCREEN_WIDTH, SCREEN_HEIGHT}
    };
    Vector2 screenRightNormal = {-1, 0}; 

    DetectLineSegmentCollision(
        &screenRight[0],
        &screenRight[1],
        &screenRightNormal,
        rigidBody
    );
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

void RunRigidBodySimulation(SDL_Renderer *renderer, float dt)
{
    for (int i = 0; i < NUM_RIGID_BODIES; ++i)
    {
        RigidBody *rigidBody = &rigidBodies[i];
        ComputeForceAndTorque(rigidBody);
        // ПОменять метод Эйлера на метод средней точки
        Vector2 linearAcceleration = (Vector2){rigidBody->force.x / rigidBody->shape.mass, rigidBody->force.y / rigidBody->shape.mass};
        rigidBody->linearVelocity.x += linearAcceleration.x * dt;
        rigidBody->linearVelocity.y += linearAcceleration.y * dt;
        rigidBody->position.x += rigidBody->linearVelocity.x * dt;
        rigidBody->position.y += rigidBody->linearVelocity.y * dt;
        float angularAcceleration = rigidBody->torque / rigidBody->shape.momentOfInertia;
        rigidBody->angularVelocity += angularAcceleration * dt;
        rigidBody->angle += rigidBody->angularVelocity * dt;
    }
    // проверка соударений
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        // DetectCollision(&rigidBodies[i]);
        DetectCollisionNew(&rigidBodies[i]);
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

    float totalSimulationTime = 1000; 
    float currentTime = 0; 
    float dt = 1.0 / 60.0; 

    // InitializeRigidBodies(renderer);
    InitializeTestRigidBodies(renderer);

    SDL_Event e;
    int quit = 0;
    while (quit == 0)
    {
        if (currentTime < totalSimulationTime)
        {
            RunRigidBodySimulation(renderer, dt);
            currentTime += dt;
            SDL_Delay(dt * 1000); // конвертируем
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