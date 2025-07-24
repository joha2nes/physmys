#include "raylib.h"
#include "raymath.h"

typedef struct Rigidbody {
    Vector3 position;
    Quaternion rotation;
    Vector3 velocity;
    Vector3 angularVelocity;
    float inverseMass;
    Matrix inverseInertiaTensor;
    // Matrix transformMatrix; based on position and rotation, can be optimized to a 4x3 matrix
    Matrix inverseInertiaTensorWorldSpace; // = MatrixMultiply(transformMatrix, inverseInertiaTensor)
    Vector3 force;
    Vector3 torque;
    float damping;
    float angularDamping;
} Rigidbody;

//

void RigidbodyInit(
    Rigidbody* rb, 
    Vector3 position, 
    Quaternion rotation, 
    float mass, 
    Matrix inertiaTensor,
    float damping, // 1 = no damping
    float angularDamping) // 1 = no damping
{ 
    rb->position = position;
    rb->rotation = rotation;
    rb->inverseMass = 1.0f / mass;
    rb->inverseInertiaTensor = MatrixInvert(inertiaTensor);
    rb->inverseInertiaTensorWorldSpace = MatrixIdentity();
    rb->velocity = Vector3Zero();
    rb->angularVelocity = Vector3Zero();
    rb->force = Vector3Zero();
    rb->torque = Vector3Zero();
    rb->damping = damping;
    rb->angularDamping = angularDamping;
}

//

void RigidbodyAddForce(Rigidbody* rb, Vector3 force) {
    rb->force = Vector3Add(rb->force, force);
}

void RigidbodyAddTorque(Rigidbody* rb, Vector3 torque) {
    rb->torque = Vector3Add(rb->torque, torque);
}

void RigidbodyAddForceAtPosition(Rigidbody* rb, Vector3 force, Vector3 position) {
    // TODO
}

Quaternion AddScaledVector(Quaternion q, Vector3 v, float dt) {
    Quaternion vq = {
        .x = v.x,
        .y = v.y,
        .z = v.z,
        .w = 0,
    };
    return QuaternionAdd(q, 
      QuaternionMultiply(QuaternionScale(vq, dt / 2.0f), q));
}
void addScaledVectorX(Quaternion* this, const Vector3 vector, float scale) {
    Quaternion q = {
        vector.x * scale,
        vector.y * scale,
        vector.z * scale,
        0
    };
    q = QuaternionMultiply(q, *this);
    this->x += q.x * 0.5f;
    this->y += q.y * 0.5f;
    this->z += q.z * 0.5f;
    this->w += q.w * 0.5f;
};

bool RigidbodyMass(const Rigidbody* rb, float* mass) {
    if (rb->inverseMass > 0) {
        *mass = 1.0f / rb->inverseMass;
        return true;
    }
    return false;
}

Matrix TransformInertiaTensor(Matrix localInertiaTensor, Quaternion rotation) {
    // Convert quaternion to rotation matrix
    Matrix rotationMatrix = QuaternionToMatrix(rotation);

    // Transpose of the rotation matrix
    Matrix rotationMatrixT = MatrixTranspose(rotationMatrix);

    // Transform the inertia tensor: R * I_local * R^T
    Matrix worldInertiaTensor = MatrixMultiply(
        MatrixMultiply(rotationMatrix, localInertiaTensor),
        rotationMatrixT
    );

    return worldInertiaTensor;
}

void TransformInertiaTensor2(
    Matrix *iitWorld,              // Output: World-space inertia tensor
    const Quaternion *q,          // Input: Rotation quaternion
    const Matrix *iitBody,        // Input: Local-space inertia tensor (4x4 matrix)
    const Matrix *rotmat)         // Input: Rotation matrix (4x4 matrix derived from quaternion)
{
    // Extract relevant elements of the rotation matrix
    float r00 = rotmat->m0, r01 = rotmat->m1, r02 = rotmat->m2;
    float r10 = rotmat->m4, r11 = rotmat->m5, r12 = rotmat->m6;
    float r20 = rotmat->m8, r21 = rotmat->m9, r22 = rotmat->m10;

    // Extract elements from the local inertia tensor
    float i00 = iitBody->m0, i01 = iitBody->m1, i02 = iitBody->m2;
    float i10 = iitBody->m4, i11 = iitBody->m5, i12 = iitBody->m6;
    float i20 = iitBody->m8, i21 = iitBody->m9, i22 = iitBody->m10;

    // Compute intermediate values
    float t4  = r00 * i00 + r01 * i10 + r02 * i20;
    float t9  = r00 * i01 + r01 * i11 + r02 * i21;
    float t14 = r00 * i02 + r01 * i12 + r02 * i22;

    float t28 = r10 * i00 + r11 * i10 + r12 * i20;
    float t33 = r10 * i01 + r11 * i11 + r12 * i21;
    float t38 = r10 * i02 + r11 * i12 + r12 * i22;

    float t52 = r20 * i00 + r21 * i10 + r22 * i20;
    float t57 = r20 * i01 + r21 * i11 + r22 * i21;
    float t62 = r20 * i02 + r21 * i12 + r22 * i22;

    // Fill in the world-space inertia tensor
    iitWorld->m0  = t4 * r00 + t9 * r01 + t14 * r02;
    iitWorld->m1  = t4 * r10 + t9 * r11 + t14 * r12;
    iitWorld->m2  = t4 * r20 + t9 * r21 + t14 * r22;
    iitWorld->m3  = 0.0f; // Unused

    iitWorld->m4  = t28 * r00 + t33 * r01 + t38 * r02;
    iitWorld->m5  = t28 * r10 + t33 * r11 + t38 * r12;
    iitWorld->m6  = t28 * r20 + t33 * r21 + t38 * r22;
    iitWorld->m7  = 0.0f; // Unused

    iitWorld->m8  = t52 * r00 + t57 * r01 + t62 * r02;
    iitWorld->m9  = t52 * r10 + t57 * r11 + t62 * r12;
    iitWorld->m10 = t52 * r20 + t57 * r21 + t62 * r22;
    iitWorld->m11 = 0.0f; // Unused

    iitWorld->m12 = 0.0f; // Unused
    iitWorld->m13 = 0.0f; // Unused
    iitWorld->m14 = 0.0f; // Unused
    iitWorld->m15 = 1.0f; // Identity for 4x4 matrix
}


void RigidbodyIntegrate(Rigidbody* rb, float dt) {
    // v' = v*d^dt + a*dt

    Vector3 accelDt = Vector3Scale(rb->force, rb->inverseMass * dt);
    rb->velocity = Vector3Scale(rb->velocity, powf(rb->damping, dt));
    rb->velocity = Vector3Add(rb->velocity, accelDt);

    Matrix worldT = QuaternionToMatrix(rb->rotation);
    // Matrix inverseInertiaTensorW = MatrixMultiply(worldT, rb->inverseInertiaTensor);
    Matrix inverseInertiaTensorW = TransformInertiaTensor(rb->inverseInertiaTensor, rb->rotation);
    // TransformInertiaTensor2(&inverseInertiaTensorW, NULL, &rb->inverseInertiaTensor, &worldT);
    Vector3 angaccel = Vector3Transform(rb->torque, inverseInertiaTensorW);
    rb->angularVelocity = Vector3Scale(rb->angularVelocity, powf(rb->angularDamping, dt));
    rb->angularVelocity = Vector3Add(rb->angularVelocity, Vector3Scale(angaccel, dt));

    Vector3 up = Vector3Transform((Vector3) {1,0,0}, worldT);
    DrawRay((Ray){.position=rb->position, .direction=up}, BLUE);

    rb->position = Vector3Add(rb->position, Vector3Scale(rb->velocity, dt));

    // addScaledVectorX(&rb->rotation, rb->angularVelocity, dt);
    rb->rotation = AddScaledVector(rb->rotation, rb->angularVelocity, dt);
    rb->rotation = QuaternionNormalize(rb->rotation);

    rb->force = (Vector3) {0};
    rb->torque = (Vector3) {0};
}
