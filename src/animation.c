#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <SDL2/SDL.h>
#ifndef M_PI
#    define M_PI 3.14159265358979323846
#endif
#define SCREEN_WIDTH 800
#define SCREEN_HEIGHT 600
#define NUM_RIGID_BODIES 7
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

void CalculateBoxInertia(BoxShape *boxShape) {
    float m = boxShape->mass;
    float w = boxShape->width;
    float h = boxShape->height;
    boxShape->momentOfInertia = m * (w * w + h * h) / 12;
}

void ComputeForceAndTorque(RigidBody *rigidBody) {
    Vector2 f = (Vector2){10, 50};
    rigidBody->force = f;
    Vector2 r = (Vector2){rigidBody->shape.width / 2, rigidBody->shape.height / 2};
    rigidBody->torque = r.x * f.y - r.y * f.x;
}

SDL_Texture *rigidBodyTextures[NUM_RIGID_BODIES];
SDL_Texture *bodyCentersTextures[NUM_RIGID_BODIES];

void InitializeRigidBodies(SDL_Renderer *renderer) {
    for (int i = 0; i < NUM_RIGID_BODIES; ++i) {
        RigidBody *rigidBody = &rigidBodies[i];
        rigidBody->position = (Vector2){rand() % (SCREEN_WIDTH - 50), rand() % (SCREEN_HEIGHT - 50)};
        rigidBody->angle = (rand() % 360) / 360.f * M_PI * 2;
        rigidBody->linearVelocity = (Vector2){0, -100};

        BoxShape shape;
        shape.mass = 1;
        shape.width = (1 + rand() % 2 )* 30;
        shape.height = (1 + rand() % 2 )* 30;
        CalculateBoxInertia(&shape);
        rigidBody->shape = shape;

        // создаем текстуры для тела
        SDL_Surface *surface = SDL_CreateRGBSurface(0, shape.width, shape.height, 32, 0, 0, 0, 0);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 255, 255));
        rigidBodyTextures[i] = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
        // текстура для центральной точки тела
        surface = SDL_CreateRGBSurface(0, 0.1*shape.width, 0.1*shape.height, 32, 0, 0, 0, 0);
        SDL_FillRect(surface, NULL, SDL_MapRGB(surface->format, 255, 0, 0));
        bodyCentersTextures[i] = SDL_CreateTextureFromSurface(renderer, surface);
        SDL_FreeSurface(surface);
    }
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
            rigidBody->angle,
            NULL,
            SDL_FLIP_NONE
        );

        SDL_Rect centerRect = {
            (int)(rigidBody->position.x - 0.05 * rigidBody->shape.width),
            (int)(rigidBody->position.y - 0.05 * rigidBody->shape.height),
            (int)(0.1 * rigidBody->shape.width),
            (int)(0.1 * rigidBody->shape.height)
        };
        SDL_RenderCopyEx(
            renderer,
            bodyCentersTextures[i],
            NULL,
            &centerRect,
            rigidBody->angle,
            NULL,
            SDL_FLIP_NONE
        );
    }
}

void CloseSDL(SDL_Window *window) {
    SDL_DestroyWindow(window);
    SDL_Quit();
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
        DetectCollision(&rigidBodies[i]);
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

    float totalSimulationTime = 1000; 
    float currentTime = 0; 
    float dt = 1.0 / 60.0; 

    InitializeRigidBodies(renderer);

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