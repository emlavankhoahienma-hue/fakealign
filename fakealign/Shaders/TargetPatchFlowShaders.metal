#include <metal_stdlib>
using namespace metal;

// Structure for passing tracked points to optical flow kernel
struct LKTrackPoint {
    float2 prevPosition;
    float2 currentPosition;
    float confidence;
    int status; // 1 = tracked, 0 = lost
};

// MARK: - Shi-Tomasi Corner Response Kernel
// Calculates the minimum eigenvalue of the structure tensor on a 3x3 window
kernel void patchFlowCornerResponseKernel(
    texture2d<float, access::read>  inTexture  [[texture(0)]],
    texture2d<float, access::write> outTexture [[texture(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint width = inTexture.get_width();
    uint height = inTexture.get_height();

    // Boundary guard for 3x3 kernel
    if (gid.x < 1 || gid.x >= width - 1 || gid.y < 1 || gid.y >= height - 1) {
        if (gid.x < width && gid.y < height) {
            outTexture.write(float4(0.0f, 0.0f, 0.0f, 1.0f), gid);
        }
        return;
    }

    // Read 3x3 neighborhood luma values
    float p00 = inTexture.read(uint2(gid.x - 1, gid.y - 1)).r;
    float p01 = inTexture.read(uint2(gid.x,     gid.y - 1)).r;
    float p02 = inTexture.read(uint2(gid.x + 1, gid.y - 1)).r;

    float p10 = inTexture.read(uint2(gid.x - 1, gid.y)).r;
    // float p11 = inTexture.read(uint2(gid.x, gid.y)).r;
    float p12 = inTexture.read(uint2(gid.x + 1, gid.y)).r;

    float p20 = inTexture.read(uint2(gid.x - 1, gid.y + 1)).r;
    float p21 = inTexture.read(uint2(gid.x,     gid.y + 1)).r;
    float p22 = inTexture.read(uint2(gid.x + 1, gid.y + 1)).r;

    // Sobel gradients
    float Ix = (p02 + 2.0f * p12 + p22) - (p00 + 2.0f * p10 + p20);
    float Iy = (p20 + 2.0f * p21 + p22) - (p00 + 2.0f * p01 + p02);

    float Ix2 = Ix * Ix;
    float Iy2 = Iy * Iy;
    float Ixy = Ix * Iy;

    // Minimum eigenvalue (Shi-Tomasi metric)
    float trace = Ix2 + Iy2;
    float diff = Ix2 - Iy2;
    float discriminant = sqrt(max(0.0f, diff * diff + 4.0f * Ixy * Ixy));
    float lambdaMin = (trace - discriminant) * 0.5f;

    outTexture.write(float4(lambdaMin, Ix, Iy, 1.0f), gid);
}

// MARK: - Pyramidal Lucas-Kanade Patch Flow Kernel
// Iterative Lucas-Kanade optical flow on local patch
kernel void patchFlowLucasKanadeKernel(
    texture2d<float, access::sample> prevTexture   [[texture(0)]],
    texture2d<float, access::sample> currTexture   [[texture(1)]],
    device LKTrackPoint*             points        [[buffer(0)]],
    constant int&                    patchRadius   [[buffer(1)]],
    constant int&                    maxIterations [[buffer(2)]],
    uint id [[thread_position_in_grid]]
) {
    constexpr sampler linearSampler(coord::normalized,
                                    address::clamp_to_edge,
                                    filter::linear);

    float2 prevPt = points[id].prevPosition;
    float2 currPt = points[id].currentPosition;

    float2 texSize = float2(prevTexture.get_width(), prevTexture.get_height());
    if (texSize.x < 1.0f || texSize.y < 1.0f) {
        points[id].status = 0;
        return;
    }

    float2 invSize = 1.0f / texSize;

    // Spatial gradient accumulation matrix G
    float gxx = 0.0f;
    float gyy = 0.0f;
    float gxy = 0.0f;

    for (int dy = -patchRadius; dy <= patchRadius; ++dy) {
        for (int dx = -patchRadius; dx <= patchRadius; ++dx) {
            float2 sampleCoord = (prevPt + float2(dx, dy)) * invSize;
            float pXplus  = prevTexture.sample(linearSampler, sampleCoord + float2(invSize.x, 0.0f)).r;
            float pXminus = prevTexture.sample(linearSampler, sampleCoord - float2(invSize.x, 0.0f)).r;
            float pYplus  = prevTexture.sample(linearSampler, sampleCoord + float2(0.0f, invSize.y)).r;
            float pYminus = prevTexture.sample(linearSampler, sampleCoord - float2(0.0f, invSize.y)).r;

            float Ix = (pXplus - pXminus) * 0.5f;
            float Iy = (pYplus - pYminus) * 0.5f;

            gxx += Ix * Ix;
            gyy += Iy * Iy;
            gxy += Ix * Iy;
        }
    }

    float det = gxx * gyy - gxy * gxy;
    if (abs(det) < 1e-6f) {
        points[id].status = 0;
        return;
    }

    float invDet = 1.0f / det;
    float2 flow = currPt - prevPt;

    // Iterative refinement
    for (int iter = 0; iter < maxIterations; ++iter) {
        float bx = 0.0f;
        float by = 0.0f;

        for (int dy = -patchRadius; dy <= patchRadius; ++dy) {
            for (int dx = -patchRadius; dx <= patchRadius; ++dx) {
                float2 coord0 = (prevPt + float2(dx, dy)) * invSize;
                float2 coord1 = (prevPt + flow + float2(dx, dy)) * invSize;

                float I0 = prevTexture.sample(linearSampler, coord0).r;
                float I1 = currTexture.sample(linearSampler, coord1).r;
                float dI = I0 - I1;

                float pXplus  = prevTexture.sample(linearSampler, coord0 + float2(invSize.x, 0.0f)).r;
                float pXminus = prevTexture.sample(linearSampler, coord0 - float2(invSize.x, 0.0f)).r;
                float pYplus  = prevTexture.sample(linearSampler, coord0 + float2(0.0f, invSize.y)).r;
                float pYminus = prevTexture.sample(linearSampler, coord0 - float2(0.0f, invSize.y)).r;

                float Ix = (pXplus - pXminus) * 0.5f;
                float Iy = (pYplus - pYminus) * 0.5f;

                bx += dI * Ix;
                by += dI * Iy;
            }
        }

        float deltaFlowX = (gyy * bx - gxy * by) * invDet;
        float deltaFlowY = (-gxy * bx + gxx * by) * invDet;

        flow += float2(deltaFlowX, deltaFlowY);

        if (deltaFlowX * deltaFlowX + deltaFlowY * deltaFlowY < 0.001f) {
            break;
        }
    }

    points[id].currentPosition = prevPt + flow;
    points[id].status = 1;
    points[id].confidence = min(1.0f, det / (gxx + gyy + 1e-4f));
}
