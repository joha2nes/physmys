#include <raymath.h>
#include <raylib.h>
#include <float.h>

typedef enum {
    COLLIDER_SPHERE,
    COLLIDER_CAPSULE,
    COLLIDER_BOX,
    COLLIDER_COUNT,
} ColliderType;

typedef struct {
    Vector3 center;
    float radius;
    //
    // Vector3 velocity;
    float mass;
} Sphere;

typedef enum {
    CAPSULE_DIRECTION_X,
    CAPSULE_DIRECTION_Y,
    CAPSULE_DIRECTION_Z,
} CapsuleDirection;

typedef struct {
    // or Vector3 start; Vector3 direction;
    Vector3 center;
    float radius;
    float height;
    CapsuleDirection direction;
} Capsule;

typedef struct {
    Vector3 center;
    Vector3 extents;
    //
    // Vector3 velocity;
    float mass;
    Quaternion rotation;
} Box;

typedef struct {
    Vector3 point;
    Vector3 normal;
    float penetration;
} Contact;

// Sphere vs Sphere

bool SphereCollision(const Sphere* s1, const Sphere* s2) {
    Vector3 delta = Vector3Subtract(s1->center, s2->center);
    float r = s1->radius + s2->radius;
    return Vector3LengthSqr(delta) < r * r;
}

// Sphere vs Capsule

bool SphereCapsuleCollision(const Sphere* s, const Capsule* c) {
    float h = c->height / 2;
    Vector3 dir =
        c->direction == CAPSULE_DIRECTION_X ? (Vector3) {1, 0, 0} :
        c->direction == CAPSULE_DIRECTION_Y ? (Vector3) {0, 1, 0} :
        c->direction == CAPSULE_DIRECTION_Z ? (Vector3) {0, 0, 1} :
        Vector3Zero();
    Vector3 start = Vector3Add(c->center,  Vector3Scale(dir, -h));

    Vector3 delta = Vector3Subtract(s->center, start);
    float dot = Vector3DotProduct(delta, dir);
    float dotClamped = Clamp(dot, 0, c->height);
    Vector3 closestPoint = Vector3Add(start,  Vector3Scale(dir, dotClamped));
    Vector3 closestPointToSphere = Vector3Subtract(s->center, closestPoint);
    float r = s->radius + c->radius;

    return Vector3LengthSqr(closestPointToSphere) < r * r;
}

// Function to find the closest points between two 3D lines
void ClosestPointTsBetweenLines(
    Vector3 p1, Vector3 d1, // Line 1: point and direction vector
    Vector3 p2, Vector3 d2, // Line 2: point and direction vector
    float *t1, float *t2 // Output: closest points on Line 1 and Line 2
) {
    Vector3 r = Vector3Subtract(p1, p2);
    float d1_d1 = Vector3LengthSqr(d1);
    float d2_d2 = Vector3LengthSqr(d2);
    float d1_d2 = Vector3DotProduct(d1, d2);
    float r_d1 = Vector3DotProduct(r, d1);
    float r_d2 = Vector3DotProduct(r, d2);

    float denom = d1_d1 * d2_d2 - d1_d2 * d1_d2;

    if (fabs(denom) < 1e-6) { // Lines are parallel or nearly parallel
        *t1 = 0;
        *t2 = r_d2 / d2_d2;
        return;
    }

    *t1 = (r_d2 * d1_d2 - r_d1 * d2_d2) / denom;
    *t2 = (r_d2 + *t1 * d1_d2) / d2_d2;
}

// Function to find the closest points between two 3D lines
void ClosestPointsBetweenLines(
    Vector3 p1, Vector3 d1, // Line 1: point and direction vector
    Vector3 p2, Vector3 d2, // Line 2: point and direction vector
    Vector3 *c1, Vector3 *c2 // Output: closest points on Line 1 and Line 2
) {
    Vector3 r = Vector3Subtract(p1, p2);
    float d1_d1 = Vector3LengthSqr(d1);
    float d2_d2 = Vector3LengthSqr(d2);
    float d1_d2 = Vector3DotProduct(d1, d2);
    float r_d1 = Vector3DotProduct(r, d1);
    float r_d2 = Vector3DotProduct(r, d2);

    float denom = d1_d1 * d2_d2 - d1_d2 * d1_d2;

    if (fabs(denom) < 1e-6) { // Lines are parallel or nearly parallel
        *c1 = p1;
        *c2 = Vector3Add(p2, Vector3Scale(d2, r_d2 / d2_d2));
        return;
    }

    float t1 = (r_d2 * d1_d2 - r_d1 * d2_d2) / denom;
    float t2 = (r_d2 + t1 * d1_d2) / d2_d2;
    ClosestPointTsBetweenLines(p1, d1, p2, d2, &t1, &t2);

    *c1 = Vector3Add(p1, Vector3Scale(d1, t1));
    *c2 = Vector3Add(p2, Vector3Scale(d2, t2));
}

// Capsule vs Capsule

bool CapsuleCollision(const Capsule* c1, const Capsule* c2) {
    float h1 = c1->height / 2;
    Vector3 d1 =
        c1->direction == CAPSULE_DIRECTION_X ? (Vector3) {1, 0, 0} :
        c1->direction == CAPSULE_DIRECTION_Y ? (Vector3) {0, 1, 0} :
        c1->direction == CAPSULE_DIRECTION_Z ? (Vector3) {0, 0, 1} :
        Vector3Zero();
    Vector3 start1 = Vector3Add(c1->center,  Vector3Scale(d1, -h1));
    
    float h2 = c2->height / 2;
    Vector3 d2 =
        c2->direction == CAPSULE_DIRECTION_X ? (Vector3) {1, 0, 0} :
        c2->direction == CAPSULE_DIRECTION_Y ? (Vector3) {0, 1, 0} :
        c2->direction == CAPSULE_DIRECTION_Z ? (Vector3) {0, 0, 1} : Vector3Zero();
    Vector3 start2 = Vector3Add(c2->center,  Vector3Scale(d2, -h2));

    float t1, t2;
    ClosestPointTsBetweenLines(start1, d1, start2, d2, &t1, &t2);

    t1 = Clamp(t1, 0, c1->height);
    t2 = Clamp(t2, 0, c2->height);

    Vector3 p1 = Vector3Add(start1, Vector3Scale(d1, t1));
    Vector3 p2 = Vector3Add(start2, Vector3Scale(d2, t2));

    Vector3 diff = Vector3Subtract(p2, p1);
    float r = c1->radius + c2->radius;

    return Vector3LengthSqr(diff) < r * r;
}

RayCollision RaycastCapsule(Ray ray, Capsule capsule) {
    float h = capsule.height / 2;
    Vector3 dir =
        capsule.direction == CAPSULE_DIRECTION_X ? (Vector3) {1, 0, 0} :
        capsule.direction == CAPSULE_DIRECTION_Y ? (Vector3) {0, 1, 0} :
        capsule.direction == CAPSULE_DIRECTION_Z ? (Vector3) {0, 0, 1} : Vector3Zero();
    Vector3 start = Vector3Add(capsule.center,  Vector3Scale(dir, h));
    Vector3 end = Vector3Add(capsule.center,  Vector3Scale(dir, -h));

    RayCollision rc = {0};
    rc = GetRayCollisionSphere(ray, start, capsule.radius);
    if (rc.hit) {
        return rc;
    }
    return GetRayCollisionSphere(ray, end, capsule.radius);
}

// Box vs Sphere

// Function to check AABB-Sphere overlap
bool BoxSphereCollision(const Box* box, const Sphere* sphere) {
    Vector3 min = Vector3Subtract(box->center, box->extents);
    Vector3 max = Vector3Add(box->center, box->extents);

    // Find the closest point on the AABB to the sphere's center
    float closestX = fmaxf(min.x, fminf(sphere->center.x, max.x));
    float closestY = fmaxf(min.y, fminf(sphere->center.y, max.y));
    float closestZ = fmaxf(min.z, fminf(sphere->center.z, max.z));
    
    // Compute the distance squared between the sphere's center and the closest point
    float dx = closestX - sphere->center.x;
    float dy = closestY - sphere->center.y;
    float dz = closestZ - sphere->center.z;
    float distanceSquared = dx * dx + dy * dy + dz * dz;
    
    // Check if the distance is less than or equal to the radius squared
    return distanceSquared < (sphere->radius * sphere->radius);
}

// Box vs Box

// Function to check AABB vs AABB overlap
bool BoxCollision(const Box* box1, const Box* box2) {
    Vector3 min1 = Vector3Subtract(box1->center, box1->extents);
    Vector3 max1 = Vector3Add(box1->center, box1->extents);

    Vector3 min2 = Vector3Subtract(box2->center, box2->extents);
    Vector3 max2 = Vector3Add(box2->center, box2->extents);

    // Check for overlap on the x-axis
    if (max1.x < min2.x || min1.x > max2.x) {
        return false;
    }
    // Check for overlap on the y-axis
    if (max1.y < min2.y || min1.y > max2.y) {
        return false;
    }
    // Check for overlap on the z-axis
    if (max1.z < min2.z || min1.z > max2.z) {
        return false;
    }
    // Overlap exists on all axes
    return true;
}

// Box vs Capsule

// Function to compute the closest point on a line segment to a given point
Vector3 ClosestPointOnLineSegment(Vector3 p, Vector3 a, Vector3 b) {
    Vector3 ab = {b.x - a.x, b.y - a.y, b.z - a.z};
    Vector3 ap = {p.x - a.x, p.y - a.y, p.z - a.z};
    
    float ab_length_squared = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
    float t = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / ab_length_squared;
    t = Clamp(t, 0.0f, 1.0f);
    
    return (Vector3) {
        a.x + t * ab.x,
        a.y + t * ab.y,
        a.z + t * ab.z
    };
}

// Function to check AABB vs Capsule overlap
bool BoxCapsuleCollision(const Box* box, const Capsule* capsule) {
    Vector3 min = Vector3Subtract(box->center, box->extents);
    Vector3 max = Vector3Add(box->center, box->extents);

    float h = capsule->height / 2;
    Vector3 dir =
        capsule->direction == CAPSULE_DIRECTION_X ? (Vector3) {1, 0, 0} :
        capsule->direction == CAPSULE_DIRECTION_Y ? (Vector3) {0, 1, 0} :
        capsule->direction == CAPSULE_DIRECTION_Z ? (Vector3) {0, 0, 1} : Vector3Zero();
    Vector3 a = Vector3Add(capsule->center,  Vector3Scale(dir, -h));
    Vector3 b = Vector3Add(capsule->center,  Vector3Scale(dir, h));

    // Clamp capsule segment endpoints to the AABB to find the closest points
    Vector3 closestP1 = {
        Clamp(a.x, min.x, max.x),
        Clamp(a.y, min.y, max.y),
        Clamp(a.z, min.z, max.z)
    };

    // Find the closest point on the capsule's line segment to the AABB
    Vector3 capsuleClosestPoint = ClosestPointOnLineSegment(closestP1, a, b);

    // Compute the squared distance from the capsule's closest point to the AABB
    float distSquared = Vector3DistanceSqr(closestP1, capsuleClosestPoint);

    // Check if the distance is less than or equal to the capsule's radius squared
    return distSquared <= (capsule->radius * capsule->radius);
}

// Function to check if a ray intersects an AABB
bool RaycastBox(Ray ray, const Box* box, float* tHit) {
    float tmin = -FLT_MAX;
    float tmax = FLT_MAX;

    Vector3 boxMin = Vector3Subtract(box->center, box->extents);
    Vector3 boxMax = Vector3Add(box->center, box->extents);

    // Check each axis (x, y, z)
    for (int i = 0; i < 3; i++) {
        float origin = (i == 0) ? ray.position.x : (i == 1) ? ray.position.y : ray.position.z;
        float direction = (i == 0) ? ray.direction.x : (i == 1) ? ray.direction.y : ray.direction.z;
        float min = (i == 0) ? boxMin.x : (i == 1) ? boxMin.y : boxMin.z;
        float max = (i == 0) ? boxMax.x : (i == 1) ? boxMax.y : boxMax.z;

        // Handle rays parallel to the planes
        if (direction == 0.0f) {
            if (origin < min || origin > max) {
                return false; // Ray is parallel and outside the AABB
            }
        } else {
            // Compute intersection times with the slabs
            float t1 = (min - origin) / direction;
            float t2 = (max - origin) / direction;

            // Swap t1 and t2 if necessary
            if (t1 > t2) {
                float temp = t1;
                t1 = t2;
                t2 = temp;
            }

            // Update tmin and tmax
            tmin = (t1 > tmin) ? t1 : tmin;
            tmax = (t2 < tmax) ? t2 : tmax;

            // If the intervals do not overlap, there is no intersection
            if (tmin > tmax) {
                return false;
            }
        }
    }

    // If tHit is provided, return the nearest intersection time
    if (tHit != NULL) {
        *tHit = tmin;
    }

    return true;
}

// Inertia tensors

Matrix BoxInertiaTensor(Box box, float mass) {
    float dx = powf(2 * box.extents.x, 2);
    float dy = powf(2 * box.extents.y, 2);
    float dz = powf(2 * box.extents.z, 2);
    float m = (1.0f / 12.0f) / mass;
    // TODO: 3x3 istället
    return (Matrix) {
        m * (dy * dz), 0, 0, 0,
        0, m * (dx * dz), 0, 0,
        0, 0, m * (dx * dy), 0,
        0, 0, 0, 1,
    };
}

// CONTACTS

bool aabb_sphere_contact(Box box, Sphere sphere, Contact* contact) {
    // Find the closest point on the box to the sphere
    Vector3 boxMin = Vector3Subtract(box.center, box.extents);
    Vector3 boxMax = Vector3Add(box.center, box.extents);

    Vector3 closestPoint = {
        Clamp(sphere.center.x, boxMin.x, boxMax.x),
        Clamp(sphere.center.y, boxMin.y, boxMax.y),
        Clamp(sphere.center.z, boxMin.z, boxMax.z),
    };

    // Calculate the vector from the closest point to the sphere center
    Vector3 toSphere = Vector3Subtract(sphere.center, closestPoint);

    float dist = Vector3Length(toSphere);

    // If the distance is less than the radius, we have a collision
    if (dist < sphere.radius) {
        contact->point = closestPoint;
        contact->normal = Vector3Normalize(toSphere);
        contact->penetration = sphere.radius - dist;
        return true;
    }

    return false;
}

bool box_sphere_contact(Box box, Sphere sphere, Contact* contact) {
    Vector3 diff = Vector3Subtract(sphere.center, box.center);

    Vector3 local_pos = Vector3RotateByQuaternion(diff, QuaternionInvert(box.rotation));
    Sphere local_sphere = sphere;
    local_sphere.center = local_pos;
    Vector3 boxWorldCenter = box.center;
    box.center = Vector3Zero();

    if (aabb_sphere_contact(box, local_sphere, contact)) {
        contact->normal = Vector3RotateByQuaternion(contact->normal, box.rotation);
        contact->point = Vector3Add(boxWorldCenter, Vector3RotateByQuaternion(contact->point, box.rotation));
        return true;
    }

    return false;
}
