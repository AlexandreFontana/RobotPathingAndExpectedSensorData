#include "include/raylib.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//instructions for movement { path1-move(20cm), rot(30), path2-move(10cm), ...}
//instructions for measurement { path1[{l, f, r}, {1, 10, 1}, {2, 7, 1}],    path2[{5, 10, 2}, {3, 5, 5}]  }
//then we let the arduino handle taking the measurement for every increment step (avoid issues with measurements at the end)

typedef struct {
    Ray origin;
    RayCollision collision;
} RaySegment;

Rectangle resizeRect(Vector2 origin, Vector2 mouse)
{
    if (origin.x < mouse.x && origin.y > mouse.y) {
        return (Rectangle) {origin.x, mouse.y, mouse.x - origin.x, origin.y - mouse.y};
    } else if (origin.x > mouse.x && origin.y < mouse.y) {
        return (Rectangle) {mouse.x, origin.y, origin.x - mouse.x, mouse.y - origin.y};
    } else if (origin.x > mouse.x && origin.y > mouse.y){
        return (Rectangle) {mouse.x, mouse.y, origin.x - mouse.x, origin.y - mouse.y};
    } else {
        return (Rectangle) {origin.x, origin.y, mouse.x - origin.x, mouse.y - origin.y};
    }
}

void drawObstacles(Rectangle *obs, int *numObs, int maxObs)
{
    static bool drawing = false;
    static Vector2 origin;
    Vector2 mousePos;

    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && *numObs < maxObs) {
        if (!drawing) {
            drawing = true;
            origin = GetMousePosition();

            obs[*numObs] = (Rectangle){origin.x, origin.y, 0, 0};
            *numObs += 1;

        } else {
            mousePos = GetMousePosition();
            obs[*numObs - 1] = resizeRect(origin, mousePos);
        }
    } else if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        drawing = false;
    } else if (IsKeyPressed(KEY_BACKSPACE) && *numObs > 0) {
        *numObs -= 1;
    }
}


void drawPath(Vector2 *path, int *pathSize, int maxPath) {

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && *pathSize < maxPath){
        Vector2 mousePos = GetMousePosition();
        path[*pathSize] = mousePos;
        *pathSize += 1;
    }

    if (IsKeyPressed(KEY_BACKSPACE) && *pathSize > 1){
        *pathSize -= 1;
    }

}


//sensorSpots: the location of sensor relative to center of robot (if that was origin). should be Vector2[3]
RaySegment*** initRays(int *numPoints, Rectangle *obs, int numObs, Vector2 *path, int pathSize, int travelDist, Vector2 *sensorSpots) {
    
    // jagged array = array of pointers which point to a multidimensional array
    // but baisically its:     allCol [path #] [point on path #] [sensor #] = RayCollision Object (has collision or not)
    // example { path1[{1,2,3}, {1,2,3}], path2[{1,2,3}, {1,2,3}, {1,2,3}] };

    RaySegment ***allCol;
    //allocate memory to outer array of pointers
    allCol = (RaySegment ***)calloc(pathSize - 1, sizeof(RaySegment **));

    //for each path
    for (int i = 0; i < pathSize - 1; i++) {
        float xdiff = path[i+1].x  - path[i].x;
        float ydiff =  path[i+1].y - path[i].y;
        float distance = sqrt(pow(xdiff, 2) + pow(ydiff, 2));
        numPoints[i] = (int) (distance / travelDist);

        allCol[i] = (RaySegment **)calloc(numPoints[i], sizeof(RaySegment *));

        //for each point along the path
        for (int j = 0; j < numPoints[i]; j++) {
            //Point = [x, y] + t (Unit direction vector) (distance between)
            //Point x = x + (t * (xdiff / distance))
            Vector2 robotPos = {path[i].x + (j * travelDist * xdiff / distance), path[i].y + (j * travelDist * ydiff / distance)};
            double robotOri;
            if (xdiff < 0) {
                robotOri = atan( ydiff / xdiff) + PI;
            } else {
                robotOri = atan( ydiff / xdiff);
            }
            

            allCol[i][j] = (RaySegment *)calloc(3, sizeof(RaySegment));

            //for each sensor at that point
            for (int sensor = 0; sensor < 3; sensor++) {
                Vector2 senOffset = sensorSpots[sensor];
                //offset point to origin, then rotate, then offset point back
                float rayXPos = (senOffset.x*cos(robotOri)) - (senOffset.y*sin(robotOri)) + robotPos.x;
                float rayYPos = (senOffset.x*sin(robotOri)) + (senOffset.y*cos(robotOri)) + robotPos.y; 

                Vector3 dirRay = {(rayXPos - robotPos.x), (rayYPos - robotPos.y), 0};

                Ray thisSensor = (Ray){(Vector3){rayXPos, rayYPos, 0}, dirRay};
                allCol[i][j][sensor].collision.distance = 1000000;
                allCol[i][j][sensor].origin = (Ray){(Vector3){rayXPos, rayYPos, 0}, dirRay};


                //for all possible collision that sensor might have
                for (int r = 0; r < numObs; r++) { 

                    Rectangle rec = obs[r];
                    Vector3 lowerBound = {rec.x, rec.y, 0};
                    Vector3 upperBound = {rec.x + rec.width, rec.y + rec.height, 0};

                    RayCollision c = GetRayCollisionBox(thisSensor, (BoundingBox){lowerBound, upperBound});

                    if (c.hit && (c.distance < allCol[i][j][sensor].collision.distance)) {
                        allCol[i][j][sensor].collision = c;
                    }
                }
            }
        }
    }
    return allCol;
}


void drawRays(RaySegment ***allCol, int *numPoints, int pathSize) {

    for (int rayPath = 0; rayPath < pathSize - 1; rayPath++) {
        RaySegment **path = allCol[rayPath];

        for (int rayPoint = 0; rayPoint < numPoints[rayPath]; rayPoint++) {
            RaySegment *point = path[rayPoint];

            for (int rayRay = 0; rayRay < 3; rayRay++) {

                RaySegment ray = point[rayRay];
                Vector2 rayEnd = {ray.collision.point.x, ray.collision.point.y};
                Vector2 rayStart = {ray.origin.position.x, ray.origin.position.y};

                if (point[rayRay].collision.hit) {
                    DrawCircle(rayEnd.x, rayEnd.y, 10, PURPLE);
                    
                    DrawLine(rayStart.x, rayStart.y, rayEnd.x, rayEnd.y, RED);
                }

                DrawCircle(rayStart.x, rayStart.y, 10, GREEN);
            }
        }
    }
}

void printMoveInstr(Vector2* path, int pathSize) {
    int conversion = 1; //cm per 100 pixels

    //from horizontal to first path
    float xdiff = 1;
    float ydiff = 0;
    float nxdiff = path[1].x - path[0].x;
    float nydiff = path[1].y - path[0].y;
    float degrees = atan2((xdiff * nydiff) - (ydiff * nxdiff) , (xdiff * nxdiff) + (ydiff * nydiff)) * 180 / PI;

    printf("{");
    printf("\"rot%f\",", degrees);

    for (int i = 0; i < pathSize - 2; i++) {

        xdiff = path[i + 1].x - path[i].x;
        ydiff = path[i + 1].y - path[i].y;
        nxdiff = path[i + 2].x - path[i + 1].x;
        nydiff = path[i + 2].y - path[i + 1].y;

        float distance = sqrt(pow(xdiff, 2) + pow(ydiff, 2));
        float cm = distance / 100 * conversion;
        printf("\"mov%f\",", cm);

        float angle = atan2((xdiff * nydiff) - (ydiff * nxdiff) , (xdiff * nxdiff) + (ydiff * nydiff));
        degrees = angle * 180 / PI;

        printf("\"rot%f\",", degrees); 
    }
    printf("\"mov%f\",", sqrt(pow(path[pathSize - 2].x - path[pathSize - 1].x, 2) + pow(path[pathSize - 2].y - path[pathSize - 1].y, 2))/ 200 * conversion);

    //from last path to horizontal
    xdiff = path[pathSize - 1].x - path[pathSize - 2].x;
    ydiff = path[pathSize - 1].y - path[pathSize - 2].y;
    nxdiff = 1;
    nydiff = 0;
    degrees = atan2((xdiff * nydiff) - (ydiff * nxdiff) , (xdiff * nxdiff) + (ydiff * nydiff)) * 180 / PI;
    printf("\"rot%f\"};\n", degrees);
    printf("number of elements: %d\n\n", 2*(pathSize - 2) + 3);
}

void simpleArduinoInstr(Vector2* path, int pathSize, RaySegment ***allCol, int* numPoints, int stepSize) {
    int conversion = 1; //cm per 100 pixels

    //from horizontal to first path
    float xdiff = 1;
    float ydiff = 0;
    float nxdiff = path[1].x - path[0].x;
    float nydiff = path[1].y - path[0].y;
    float degrees = atan2((xdiff * nydiff) - (ydiff * nxdiff) , (xdiff * nxdiff) + (ydiff * nydiff)) * 180 / PI;

    printf("{");
    printf("\"rot%f\"", degrees);

    for (int i = 0; i < pathSize - 1; i++) {
        RaySegment **rayPath = allCol[i];
        
        xdiff = path[i + 1].x - path[i].x;
        ydiff = path[i + 1].y - path[i].y;
 
        float distance = sqrt(pow(xdiff, 2) + pow(ydiff, 2));

        for (int j = 0; j < numPoints[i]; j++) {
            
            RaySegment *rayPoint = rayPath[j];
            
            if (rayPoint[0].collision.hit) printf(", \"sn0%f\"", rayPoint[0].collision.distance);
        
            if (rayPoint[1].collision.hit) printf(", \"sn1%f\"", rayPoint[1].collision.distance);

            if (rayPoint[2].collision.hit) printf(", \"sn2%f\"", rayPoint[2].collision.distance);
            
            float stepCm = stepSize / 100.0 * conversion;
            
            printf(", \"mov%f\"", stepCm);
        }
        
        //the last bit of distance
        if (remainder(distance, stepSize) > 0.001) printf(", \"mov%f\"", remainder(distance, stepSize) / 100 * conversion);

        nxdiff = path[i + 2].x - path[i + 1].x;
        nydiff = path[i + 2].y - path[i + 1].y;

        float angle = atan2((xdiff * nydiff) - (ydiff * nxdiff) , (xdiff * nxdiff) + (ydiff * nydiff));
        degrees = angle * 180 / PI;

        printf(", \"rot%f\"", degrees); 
    
    }
    printf("};\n\n");
}



enum Mode {
    PATH,
    OBS,
    RAY,
};

int main(void)
{
    const int screenWidth = 1280;
    const int screenHeight = 720;

    InitWindow(screenWidth, screenHeight, "Robot Path");

    enum Mode mode = RAY; 

    Vector2 startPos = {0, screenHeight / 2};
    Vector2 startSize = {100, 50};
    Vector2 sensorPos[3] = {(Vector2){ 0, -20}, (Vector2){ 0, 20}, (Vector2){20, 0}};

    int maxPath = 40;
    Vector2 path[maxPath];
    path[0] = (Vector2) {startPos.x + (startSize.x / 2), startPos.y + (startSize.y / 2)};
    int pathSize = 1;

    int maxObs = 40;
    Rectangle obstacles[maxObs];
    int numObs = 0;

    int travelDist = 50; //distance between distance measurements (in pixels)
    int numPoints[pathSize]; //number of points on each path.
    RaySegment ***allCol;

    SetTargetFPS(60);

    while (!WindowShouldClose()){
        BeginDrawing();

            ClearBackground(GRAY);
            DrawRectangleV(startPos, startSize, RED);

            if (IsKeyPressed(KEY_A)) mode = PATH;
            else if (IsKeyPressed(KEY_S)) mode = OBS;
            else if (IsKeyPressed(KEY_F)) {
                allCol = initRays(numPoints, obstacles, numObs, path, pathSize, travelDist, sensorPos);
                simpleArduinoInstr(path, pathSize, allCol, numPoints, travelDist);
            } else if (IsKeyPressed(KEY_D)) {
                mode = RAY;
                allCol = initRays(numPoints, obstacles, numObs, path, pathSize, travelDist, sensorPos);
            } 

            for (int i = 0; i < pathSize; i++){
                if (i < pathSize - 1){
                    DrawLineV(path[i], path[i + 1], GREEN);
                }

                DrawCircleV(path[i], 10, BLACK);
            }

            for (int i = 0; i < numObs; i++){
                DrawRectangleRec(obstacles[i], GREEN);
                DrawRectangleLinesEx(obstacles[i], 5, BLACK);
            }

            switch (mode) {
                case PATH: drawPath(path, &pathSize, maxPath); break;
                case OBS: drawObstacles(obstacles, &numObs, maxObs); break;
                default: drawRays(allCol, numPoints, pathSize);
            }


        EndDrawing();
    }

    CloseWindow();

    return 0;
}
