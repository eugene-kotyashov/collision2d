#ifndef GEOM_UTILS_H
#define GEOM_UTILS_H


typedef struct {
    float x;
    float y;
} Vector2;

typedef struct {
    float x;
    float y; 
    float z;
} Vector3;


float dotProduct(Vector2* v1, Vector2* v2);

Vector3 crossProduct(Vector3 w, Vector3 v);

float length(Vector2* v);

Vector2 normalize(Vector2* v);

float pointToLineDistance(
    Vector2 pt1, Vector2 pt2, Vector2 p, Vector2* pointToLineV);

Vector2 rotateVector(Vector2* inV, float angleRad);

void testGeometryFunctions();

#endif