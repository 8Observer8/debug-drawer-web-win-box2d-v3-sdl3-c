#define SDL_MAIN_USE_CALLBACKS 1 // Use callbacks instead of main()
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <box2d/box2d.h>
#include <math.h>
#include <stdio.h>

static SDL_Window *window = NULL;
static SDL_Renderer *renderer = NULL;

b2WorldId worldId;           // Physical world identifier
float pixelsPerMeter = 30.f; // To recalculate coordinates and sizes: world <-> pixels
b2DebugDraw debugDrawer;     // For drawing colliders during debugging
b2BodyId racketBodyId;       // Racket body
b2BodyId ballBodyId;         // Ball body
bool keyLeft = false;        // The "left" key is pressed
bool keyRight = false;       // The right key is pressed
float racketSpeed = 5.f;     // Racket speed

void initPhysicsWorld(void);
void drawSolidPolygon(b2Transform transform, const b2Vec2* vertices,
    int vertexCount, float radius, b2HexColor color, void* context);
void drawSolidCircle(b2Transform transform, float radius,
    b2HexColor color, void* context);

// ------------------------------------------------------------
SDL_AppResult SDL_AppInit(void **appstate, int argc, char *argv[])
{
    SDL_SetAppMetadata("Arkanoid using Box2D v3, SDL3, and C", "1.0",
        "com.example.arkanoid");

    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    if (!SDL_CreateWindowAndRenderer("Arkanoid using Box2D v3, SDL3, and C",
        400, 300, 0, &window, &renderer))
    {
        SDL_Log("Couldn't create window/renderer: %s", SDL_GetError());
        return SDL_APP_FAILURE;
    }

    // SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
    SDL_SetRenderVSync(renderer, 1); // Turn on vertical sync
    initPhysicsWorld();              // Initializing the physical world
    // Create a collider line drawer for debugging
    debugDrawer = b2DefaultDebugDraw();
    debugDrawer.drawShapes = true;
    debugDrawer.DrawSolidPolygon = drawSolidPolygon;
    debugDrawer.DrawSolidCircle = drawSolidCircle;

    return SDL_APP_CONTINUE;
}
// ------------------------------------------------------------
SDL_AppResult SDL_AppEvent(void *appstate, SDL_Event *event)
{
    if (event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS;
    }
    else if (event->type == SDL_EVENT_KEY_DOWN) {
        if (event->key.scancode == SDL_SCANCODE_LEFT)
            keyLeft = true;
        else if(event->key.scancode == SDL_SCANCODE_RIGHT)
            keyRight = true;
    } else if (event->type == SDL_EVENT_KEY_UP) {
        if (event->key.scancode == SDL_SCANCODE_LEFT)
            keyLeft = false;
        else if(event->key.scancode == SDL_SCANCODE_RIGHT)
            keyRight = false;
    }
    return SDL_APP_CONTINUE;
}
// ------------------------------------------------------------
uint32_t lastTickTime = 0;
// ------------------------------------------------------------
SDL_AppResult SDL_AppIterate(void *appstate)
{
    uint32_t currentTickTime = SDL_GetTicks();
    float deltaTime = (currentTickTime - lastTickTime) / 1000.f;
    lastTickTime = currentTickTime;

    // Racket control
    if (keyLeft) {
        float x = b2Body_GetPosition(racketBodyId).x;
        float y = b2Body_GetPosition(racketBodyId).y;
        x = x - racketSpeed * deltaTime;
        if (x * pixelsPerMeter > 50) {
            b2Body_SetTransform(racketBodyId, (b2Vec2){x, y}, b2MakeRot(0.f));
            // b2Body_SetTransform(racketBodyId, (b2Vec2){x, y}, (b2Rot){1.f, 0.f});
        }
    } else if (keyRight) {
        float x = b2Body_GetPosition(racketBodyId).x;
        float y = b2Body_GetPosition(racketBodyId).y;
        x = x + racketSpeed * deltaTime;
        if (x * pixelsPerMeter < 350) {
            b2Body_SetTransform(racketBodyId, (b2Vec2){x, y}, b2MakeRot(0.f));
        }
    }

    b2Vec2 velBefore = b2Body_GetLinearVelocity(ballBodyId);

    b2World_Step(worldId, 0.016, 5); // Make a time step of 0.016 sec

    b2ContactEvents contactEvents = b2World_GetContactEvents(worldId);

    if (contactEvents.beginCount > 0) {
        printf("collision\n");
        fflush(stdout);
    }

    SDL_SetRenderDrawColor(renderer, 33, 33, 33, SDL_ALPHA_OPAQUE); // Canvas color
    SDL_RenderClear(renderer); // Clear the canvas and fill it with the canvas color

    b2World_Draw(worldId, &debugDrawer); // Drawing collider lines for debugging

    SDL_RenderPresent(renderer);  // Display the contents of the drawer on the screen
    return SDL_APP_CONTINUE;  // Continue running the program!
}
// ------------------------------------------------------------
void SDL_AppQuit(void *appstate, SDL_AppResult result)
{
    // Window and renderer will be automatically removed
    // Deleting the physical world will also delete bodies and shapes
    b2DestroyWorld(worldId);
    worldId = b2_nullWorldId;
}
// ------------------------------------------------------------
void initPhysicsWorld(void)
{
    // Create a physical world with zero gravity
    b2Vec2 gravity = { 0.f, 0.f };
    b2WorldDef worldDef = b2DefaultWorldDef();
    worldDef.gravity = gravity;
    worldId = b2CreateWorld(&worldDef);

    b2World_SetRestitutionThreshold(worldId, 0.1f);

    // Create walls
    // Left wall
    b2BodyDef leftWallBodyDef = b2DefaultBodyDef();
    leftWallBodyDef.type = b2_staticBody;
    leftWallBodyDef.position = (b2Vec2){ 10.f / pixelsPerMeter,
        150.f / pixelsPerMeter };
    b2BodyId leftWallBodyId = b2CreateBody(worldId, &leftWallBodyDef);
    // Left wall shape
    b2Polygon leftWallShape = b2MakeBox(10.f / pixelsPerMeter, 150.f / pixelsPerMeter);
    b2ShapeDef leftWallShapeDef = b2DefaultShapeDef();
    leftWallShapeDef.friction = 0.f;
    b2CreatePolygonShape(leftWallBodyId, &leftWallShapeDef, &leftWallShape);
    // Right wall
    b2BodyDef rightWallBodyDef = b2DefaultBodyDef();
    rightWallBodyDef.type = b2_staticBody;
    rightWallBodyDef.position = (b2Vec2){ 390.f / pixelsPerMeter, 150.f / pixelsPerMeter };
    b2BodyId rightWallBodyId = b2CreateBody(worldId, &rightWallBodyDef);
    // Right wall shape
    b2Polygon rightWallShape = b2MakeBox(10.f / pixelsPerMeter, 150.f / pixelsPerMeter);
    b2ShapeDef rightWallShapeDef = b2DefaultShapeDef();
    rightWallShapeDef.friction = 0.f;
    b2CreatePolygonShape(rightWallBodyId, &rightWallShapeDef, &rightWallShape);
    // Top wall
    b2BodyDef topWallBodyDef = b2DefaultBodyDef();
    topWallBodyDef.type = b2_staticBody;
    topWallBodyDef.position = (b2Vec2){ 200.f / pixelsPerMeter, 10.f / pixelsPerMeter };
    b2BodyId topWallBodyId = b2CreateBody(worldId, &topWallBodyDef);
    // Top wall shape
    b2Polygon topWallShape = b2MakeBox(180.f / pixelsPerMeter, 10.f / pixelsPerMeter);
    b2ShapeDef topWallShapeDef = b2DefaultShapeDef();
    topWallShapeDef.friction = 0.f;
    b2CreatePolygonShape(topWallBodyId, &topWallShapeDef, &topWallShape);
    // Bottom wall
    b2BodyDef bottomWallBodyDef = b2DefaultBodyDef();
    bottomWallBodyDef.type = b2_staticBody;
    bottomWallBodyDef.position = (b2Vec2){ 200.f / pixelsPerMeter, 290.f / pixelsPerMeter };
    b2BodyId bottomWallBodyId = b2CreateBody(worldId, &bottomWallBodyDef);
    // Top wall shape
    b2Polygon bottomWallShape = b2MakeBox(180.f / pixelsPerMeter, 10.f / pixelsPerMeter);
    b2ShapeDef bottomWallShapeDef = b2DefaultShapeDef();
    bottomWallShapeDef.friction = 0.f;
    b2CreatePolygonShape(bottomWallBodyId, &bottomWallShapeDef, &bottomWallShape);

    // Create a racket
    b2BodyDef racketBodyDef = b2DefaultBodyDef();
    racketBodyDef.type = b2_kinematicBody;
    racketBodyDef.position = (b2Vec2){ 200.f / pixelsPerMeter, 265.f / pixelsPerMeter };
    racketBodyId = b2CreateBody(worldId, &racketBodyDef);
    // Racket shape
    b2Polygon racketShape = b2MakeBox(30.f / pixelsPerMeter, 5.f / pixelsPerMeter);
    b2ShapeDef racketShapeDef = b2DefaultShapeDef();
    racketShapeDef.friction = 0.f;
    b2ShapeId racketShapeId = b2CreatePolygonShape(racketBodyId, &racketShapeDef, &racketShape);
    b2Shape_SetUserData(racketShapeId, "racket");

    // Create a ball
    b2BodyDef ballBodyDef = b2DefaultBodyDef();
    ballBodyDef.type = b2_dynamicBody;
    ballBodyDef.position = (b2Vec2){200.f / pixelsPerMeter, 250.f / pixelsPerMeter};
    ballBodyId = b2CreateBody(worldId, &ballBodyDef);
    // -----
    // Ball shape (box)
    // b2Polygon ballShape = b2MakeBox(5.f / pixelsPerMeter, 5.f / pixelsPerMeter);
    // -----
    // Ball shape (circle)
    b2Circle ballShape = {0};
    ballShape.radius = 5.f / pixelsPerMeter;
    // -----
    b2ShapeDef ballShapeDef = b2DefaultShapeDef();
    ballShapeDef.enableContactEvents = true;
    ballShapeDef.friction = 0.f;
    ballShapeDef.restitution = 1.f;
    // -----
    // Ball shape (box)
    // b2ShapeId ballShapeId = b2CreatePolygonShape(ballBodyId, &ballShapeDef, &ballShape);
    // -----
    // Ball shape (circle)
    b2ShapeId ballShapeId = b2CreateCircleShape(ballBodyId, &ballShapeDef, &ballShape);
    // -----
    b2Body_SetLinearVelocity(ballBodyId, (b2Vec2){0.f, -2.9f}); // Initial velocity
    b2Body_SetFixedRotation(ballBodyId, true); // Do not rotate a ball
    // Store the name of the object in its physical body
    // strcpy(ballUserData.name, "ball");
    // b2Body_SetUserData(ballBodyId, &ballUserData);
    b2Shape_SetUserData(ballShapeId, "ball"); // myEntityInfo

    // Create the first block
    b2BodyDef block0BodyDef = b2DefaultBodyDef();
    block0BodyDef.type = b2_staticBody;
    block0BodyDef.position = (b2Vec2){ 206.f / pixelsPerMeter, 120.f / pixelsPerMeter };
    block0BodyDef.rotation = b2MakeRot(85.f * 3.14f / 180.f);
    b2BodyId block0BodyId = b2CreateBody(worldId, &block0BodyDef);
    // Block shape
    b2Polygon block0Shape = b2MakeBox(50.f / pixelsPerMeter, 5.f / pixelsPerMeter);
    // b2Vec2 block0Points[] = {
    //     (b2Vec2) { -20.f / pixelsPerMeter, -10.f / pixelsPerMeter },
    //     (b2Vec2) { -20.f / pixelsPerMeter, 10.f / pixelsPerMeter },
    //     (b2Vec2) { 20.f / pixelsPerMeter, 10.f / pixelsPerMeter },
    //     (b2Vec2) { 20.f / pixelsPerMeter, -10.f / pixelsPerMeter }
    // };
    // b2Hull block0Hull = b2ComputeHull(block0Points, 4);
    // b2Polygon block0Shape = b2MakePolygon(&block0Hull, 0.f);
    b2ShapeDef block0ShapeDef = b2DefaultShapeDef();
    block0ShapeDef.friction = 0.f;
    b2ShapeId block0ShapeId = b2CreatePolygonShape(block0BodyId, &block0ShapeDef, &block0Shape);
    b2Shape_SetUserData(block0ShapeId, "block");
}
// ------------------------------------------------------------
// Drawing lines around Box2D colliders for debugging
void drawSolidPolygon(b2Transform transform, const b2Vec2* vertices,
    int vertexCount, float radius, b2HexColor color, void* context)
{
    // Get the pixel format
    SDL_Surface *surface = SDL_GetWindowSurface(window);
    const SDL_PixelFormatDetails *format = SDL_GetPixelFormatDetails(surface->format);
    // Extract RGB
    Uint8 r, g, b;
    SDL_GetRGB(color, format, NULL, &r, &g, &b);
    // Draw a collider rectangle with lines
    SDL_SetRenderDrawColor(renderer, r, g, b, SDL_ALPHA_OPAQUE);
    for (int i = 0; i < vertexCount; ++i) {
        int next_index = (i + 1 == vertexCount) ? 0 : i + 1;
        b2Vec2 p0 = b2TransformPoint(transform, vertices[i]);
        b2Vec2 p1 = b2TransformPoint(transform, vertices[next_index]);
        float x0 = p0.x * pixelsPerMeter;
        float y0 = p0.y * pixelsPerMeter;
        float x1 = p1.x * pixelsPerMeter;
        float y1 = p1.y * pixelsPerMeter;
        SDL_RenderLine(renderer, x0, y0, x1, y1);
    }
}
// ------------------------------------------------------------
void drawSolidCircle(b2Transform transform, float radius,
    b2HexColor color, void* context)
{
    float angle = 0.f;
    const int numberOfSegments = 20;
    const float angleStep = 360.f / numberOfSegments;

    // Get the pixel format
    SDL_Surface *surface = SDL_GetWindowSurface(window);
    const SDL_PixelFormatDetails *format = SDL_GetPixelFormatDetails(surface->format);
    // Extract RGB
    Uint8 r, g, b;
    SDL_GetRGB(color, format, NULL, &r, &g, &b);
    // Draw a collider rectangle with lines
    SDL_SetRenderDrawColor(renderer, r, g, b, SDL_ALPHA_OPAQUE);

    float x = radius * cos(angle * 3.14f / 180.f);
    float y = radius * sin(angle * 3.14f / 180.f);
    b2Vec2 p0 = b2TransformPoint(transform, (b2Vec2){ x, y });
    float x0 = p0.x * pixelsPerMeter;
    float y0 = p0.y * pixelsPerMeter;
    angle += angleStep;

    for (int i = 0; i < numberOfSegments; ++i) {
        float x = radius * cos(angle * 3.14f / 180.f);
        float y = radius * sin(angle * 3.14f / 180.f);
        b2Vec2 p1 = b2TransformPoint(transform, (b2Vec2){ x, y });
        float x1 = p1.x * pixelsPerMeter;
        float y1 = p1.y * pixelsPerMeter;
        SDL_RenderLine(renderer, x0, y0, x1, y1);
        x0 = x1;
        y0 = y1;
        angle += angleStep;
        if (angle >= 360.f) {
            angle = 0.f;
        }
    }
}
// ------------------------------------------------------------
