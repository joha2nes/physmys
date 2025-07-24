#include <stdio.h>
#include <dlfcn.h>

#include "raylib.h"
#include "raymath.h"

#include "list.h"
#include "colliders.h"
#include "rigidbody.h"

#include "debug_draw.h"

#define debug_log_f(x) printf("%s: %f\n", #x, x)
#define debug_log_zu(x) printf("%s: %zu\n", #x, x)
#define debug_log_i(x) printf("%s: %i\n", #x, x)
#define debug_log_v(_x) printf("%s: {%f, %f, %f}\n", #_x, _x.x, _x.y, _x.z)
#define debug_log_s(x) printf("%s\n", x)

float min_abs(float x, float y) {
    return fabsf(x) < fabsf(y) ? x : y;
}

float max_abs(float x, float y) {
    return fabsf(x) > fabsf(y) ? x : y;
}

Vector3 Vector3ProjectOnPlane(Vector3 v, Vector3 normal) {
    float dot = Vector3DotProduct(v, normal);
    return Vector3Subtract(v, Vector3Scale(normal, dot));
}

Quaternion ApplyAngularVelocity(Quaternion q, Vector3 angularVelocity, float deltaTime) {
    // Convert angular velocity to a pure quaternion (0, wx, wy, wz)
    Quaternion omegaQ = { angularVelocity.x, angularVelocity.y, angularVelocity.z, 0 };

    // Compute quaternion derivative: dQ/dt = 0.5 * omegaQ * Q
    Quaternion qDot = QuaternionMultiply(omegaQ, q);
    qDot.x *= 0.5f;
    qDot.y *= 0.5f;
    qDot.z *= 0.5f;
    qDot.w *= 0.5f;

    // Integrate: Q_new = Q + dQ * dt
    q.x += qDot.x * deltaTime;
    q.y += qDot.y * deltaTime;
    q.z += qDot.z * deltaTime;
    q.w += qDot.w * deltaTime;

    // Normalize to prevent drift
    return QuaternionNormalize(q);
}

int main(void)
{
    InitWindow(1024, 720, "Template-5.0");

    SetWindowFocused();

    mk_list(boxes, Box, 12);
    {
        #define add_box(_x) { list_add(&boxes, (_x)); }
        add_box(((Box) {
            .center = {0,0,10},
            .rotation = QuaternionFromAxisAngle((Vector3) {1, 0, 0}, DEG2RAD * -30),
            .extents = {10, 10, 40},
            .mass = 5
        }));
        // add_box(((Box) {
        //     .center = {0, 00, -30},
        //     .rotation = QuaternionFromAxisAngle((Vector3) {1, 0.5, -0.2}, DEG2RAD * 45),
        //     .extents = {20, 10, 20},
        //     .mass = 5
        // }));
    }

    Vector3 pos0 = {0, 30, 0};
    Vector3 vel0 = {0, 0, 0};
    Vector3 avel0 = {0, 0, 0};
    Sphere marble = {
        .center = pos0,
        .mass = 5,
        .radius = 10
    };

    Vector3 marble_vel = vel0;
    Vector3 marble_avel = avel0;
    Quaternion marble_rot = QuaternionIdentity();

    Vector3 gravity = {0, -39.82, 0};

    Mesh cube_mesh = GenMeshCube(1, 1, 1);
    Model cube_model = LoadModelFromMesh(cube_mesh);
    Material cube_mat = LoadMaterialDefault();
    Texture2D tex = LoadTexture("assets/screenshot.png");
    SetMaterialTexture(&cube_mat, TEXTURE_FILTER_POINT, tex);
    cube_model.materials[0] = cube_mat;

    Mesh sphere_mesh = GenMeshSphere(1, 20, 32);
    Model sphere_model = LoadModelFromMesh(sphere_mesh);
    Material sphere_mat = LoadMaterialDefault();
    SetMaterialTexture(&sphere_mat, TEXTURE_FILTER_POINT, tex);
    sphere_model.materials[0] = sphere_mat;

    Camera3D cam = {0};
    cam.fovy = 90;
    cam.projection = CAMERA_PERSPECTIVE;
    cam.position = (Vector3) {43.177147, 22.953009, -27.693853};
    cam.up = (Vector3) {0.000000, 1.000000, 0.000000};
    cam.target = (Vector3) {-36.849541, -9.439035, 22.769011};

    while (!WindowShouldClose())
    {
        if (IsKeyPressed(KEY_R)) {
            marble.center = pos0;
            marble_vel = vel0;
            marble_avel = avel0;
        }

        // UpdateCamera(&cam, CAMERA_FREE);

        float dt = GetFrameTime();
        if (IsKeyDown(KEY_F)) {
            dt *= 5;
        }

        //
        if (IsKeyPressed(KEY_SPACE)) {
            marble_vel = Vector3Add(marble_vel, (Vector3){0, 30, 0});
        }
        Vector3 torque = IsKeyDown(KEY_LEFT) ? (Vector3){ 4000, 0, 0 } : IsKeyDown(KEY_RIGHT) ? (Vector3){ -4000, 0, 0 } : Vector3Zero();
        float intertia = (2.0f / 5.0f) * marble.mass * marble.radius * marble.radius;
        Vector3 angular_acceleration;
        angular_acceleration.x = torque.x / intertia;
        angular_acceleration.y = torque.y / intertia;
        angular_acceleration.z = torque.z / intertia;
        marble_avel = Vector3Add(marble_avel, Vector3Scale(angular_acceleration, dt));
        //

        marble_vel = Vector3Add(marble_vel, Vector3Scale(gravity, dt));
        marble.center = Vector3Add(marble.center, Vector3Scale(marble_vel, dt));
        marble_rot = ApplyAngularVelocity(marble_rot, marble_avel, dt);

        Vector3 col = {0};
        Vector3 coln = {0};
        Vector3 colp = {0};
        int col_count = 0;
        for (int i = 0; i < boxes.count; i++) {
            Contact c;
            if (box_sphere_contact(boxes.items[i], marble, &c)) {
                Vector3 v = Vector3Scale(c.normal, c.penetration);
                col.x = max_abs(col.x, v.x);
                col.y = max_abs(col.y, v.y);
                col.z = max_abs(col.z, v.z);
                col_count++;
                coln = Vector3Add(coln, c.normal);

                marble.center = Vector3Add(marble.center, col);
                colp = c.point;
            }
        }

        if (col_count > 0) {
            coln.x /= col_count;
            coln.y /= col_count;
            coln.z /= col_count;
            coln = Vector3Normalize(coln);

            // debug_draw_ray((Ray) {colp, coln}, RED, 1);

            float bounciness = 0.6;
            float static_friction = 0.2;
            float dynamic_friction = 0.4;

            Vector3 reflected_vel = Vector3Reflect(marble_vel, coln);
            Vector3 planal_vel = Vector3ProjectOnPlane(reflected_vel, coln);
            // Vector3 normal_vel = Vector3Scale(Vector3Project(reflected_vel, coln), bounciness);
            // Vector3 bounce_vel = Vector3Add(planal_vel, normal_vel); // this is the perfect bounce velocity if there were no friction on the collision

            // Vector3 vel_at_contact = Vector3Add(marble_vel, Vector3CrossProduct(marble_avel, Vector3Scale(coln, -marble.radius)));

            Vector3 force = {0};
            
            Vector3 vel_against_normal = Vector3Project(marble_vel, coln);
            Vector3 stop_impulse = Vector3Scale(Vector3Negate(vel_against_normal), marble.mass);
            Vector3 stop_force = Vector3Divide(stop_impulse, (Vector3) {dt,dt,dt});
            force = Vector3Add(force, stop_force);
            
            Vector3 bounce_impulse = Vector3Scale(Vector3Negate(vel_against_normal), bounciness * marble.mass);
            Vector3 bounce_force = Vector3Divide(bounce_impulse, (Vector3) {dt,dt,dt});
            force = Vector3Add(force, bounce_force);

            // Vector3 avel_vel = Vector3CrossProduct(Vector3Scale(marble_avel, marble.radius), coln);
            Vector3 target_avel = Vector3Divide(Vector3CrossProduct(coln, planal_vel), (Vector3){marble.radius,marble.radius,marble.radius});
            Vector3 diff_avel = Vector3Subtract(marble_avel, target_avel);
            Vector3 diff_avel_vel = Vector3CrossProduct(Vector3Scale(diff_avel, marble.radius), coln);
            // TODO: använd stop_force för att avgöra static eller dynamic friction?
            Vector3 static_friction_impulse = Vector3Scale(diff_avel_vel, static_friction * marble.mass);
            Vector3 static_friction_force = Vector3Divide(static_friction_impulse, (Vector3) {dt,dt,dt});
            force = Vector3Add(force, static_friction_force);

            Vector3 force_accel = Vector3Divide(force, (Vector3){marble.mass, marble.mass, marble.mass});

            marble_vel = Vector3Add(marble_vel, Vector3Scale(force_accel, dt));

            Vector3 planal_vel2 = Vector3ProjectOnPlane(marble_vel, coln);
            Vector3 target_avel2 = Vector3CrossProduct(coln, planal_vel2);
            target_avel2.x /= marble.radius;
            target_avel2.y /= marble.radius;
            target_avel2.z /= marble.radius;

            marble_avel = target_avel2;
        }

        BeginDrawing();
        {
            BeginMode3D(cam);
            {
                ClearBackground(RAYWHITE);
                DrawGrid(10, 10);

                // Draw

                Color marble_color = col_count > 0 ? YELLOW : BLUE;
                sphere_model.transform = QuaternionToMatrix(marble_rot);
                DrawModelEx(sphere_model, marble.center, (Vector3) {0,1,0}, 0, Vector3Scale(Vector3One(), marble.radius), marble_color);

                // DrawRay((Ray) { .position = marble.center, .direction = Vector3Scale(Vector3Normalize(marble_vel), 30) }, BLACK);
                // DrawRay((Ray) { .position = marble.center, .direction = Vector3Scale(coln, 30) }, RED);
                // DrawRay((Ray){.position = colp, .direction = marble_avel}, MAGENTA);

                for (int i = 0; i < boxes.count; i++)
                {
                    Box b = boxes.items[i];
                    Color color = WHITE;
                    
                    Matrix matrixT = MatrixTranslate(b.center.x, b.center.y, b.center.z);
                    Matrix matrixR = QuaternionToMatrix(b.rotation);
                    Matrix matrixS =  MatrixScale(2 * b.extents.x, 2 * b.extents.y, 2 * b.extents.z);
                    cube_model.transform = MatrixMultiply(MatrixMultiply(matrixS, matrixR), matrixT);
                    
                    DrawModelEx(cube_model, Vector3Zero(), Vector3Zero(), 0, Vector3One(), color);
                }

                DrawRay((Ray) { .position = {0,0,20},.direction = {10, 0, 0}}, RED);
                DrawRay((Ray) { .position = {0,0,20},.direction = {0, 10, 0}}, GREEN);
                DrawRay((Ray) { .position = {0,0,20},.direction = {0, 0, 10}}, BLUE);

                debug_draw_update();
            }
            EndMode3D();

            Vector2 screenSize = {
                GetScreenWidth(),
                GetScreenHeight()
            };

            DrawCircle(screenSize.x / 2, screenSize.y / 2, 3, BLACK);
        }
        EndDrawing();
    }
    
    debug_log_v(cam.position);
    debug_log_v(cam.up);
    debug_log_v(cam.target);

    CloseWindow();

    return 0;
}