#include <math.h>
#include <stdlib.h>
#include "geom_utils.h"

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
    // printf("v1:(%e, %e) v2:(%e, %e)\n", v1.x, v1.y, v2.x, v2.y);
    // printf("dot %e\n", dot);
    // printf("nor v1:(%e, %e)\n", v1norm.x, v1norm.y);
    Vector2 origin = {0, 0};
    Vector2 xAxis = {3, 0};
    Vector2 yAxis = {0, 10};
    Vector2 pt1 = { 1, 1};
    /*
    printf(
        "pt1 both dists has to be 1, actial is %e %e\n ",
        pointToLineDistance(origin, xAxis, pt1, NULL),
        pointToLineDistance(origin, yAxis, pt1, NULL)
    );
    */
    Vector2 normal = {1, 1};
    normal = normalize(&normal);
    float len = sqrt(dotProduct(&normal, &normal));
    normal = rotateVector(&normal, 0.25*M_PI);
    float lenRot = sqrt(dotProduct(&normal, &normal));
    // printf("len %.2e rotated len %.2e\n", len, lenRot);

    Vector3 i = { 1, 0, 0}; 
    Vector3 j = {0, 1, 0};
    Vector3 k = crossProduct(i, j); 
    // printf("cross resul (%.4e %.4e %.4e) expected (0, 0, 1)\n", k.x, k.y, k.z );
}