#pragma once

namespace skein::shaders {

constexpr int MAX_POINT_LIGHTS = 32;

inline const char* LIT_VERTEX = R"(#version 410 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;
layout(location = 3) in mat4 aModel;

uniform mat4 uViewProj;
uniform mat4 uLightViewProj;

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUv;
out vec4 vLightSpace;

void main() {
    vec4 world = aModel * vec4(aPosition, 1.0);
    vWorldPos = world.xyz;
    vNormal = transpose(inverse(mat3(aModel))) * aNormal;
    vUv = aUv;
    vLightSpace = uLightViewProj * world;
    gl_Position = uViewProj * world;
}
)";

inline const char* LIT_FRAGMENT = R"(#version 410 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec2 vUv;
in vec4 vLightSpace;

out vec4 fragColor;

uniform vec3 uCameraPos;
uniform vec3 uAlbedo;
uniform vec3 uEmissive;
uniform float uRoughness;
uniform float uMetallic;

uniform vec3 uSunDirection;
uniform vec3 uSunColor;
uniform float uSunIntensity;
uniform vec3 uAmbientSky;
uniform vec3 uAmbientGround;

uniform int uPointCount;
uniform vec4 uPointPosRange[32];
uniform vec4 uPointColor[32];

uniform sampler2D uShadowMap;
uniform float uShadowTexel;
uniform int uShadowEnabled;
uniform float uFogDensity;
uniform vec3 uFogColor;

float shadowFactor(vec3 normal, vec3 lightDir) {
    if (uShadowEnabled == 0) return 1.0;
    vec3 proj = vLightSpace.xyz / vLightSpace.w;
    proj = proj * 0.5 + 0.5;
    if (proj.z > 1.0 || proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0) return 1.0;
    float bias = max(0.0025 * (1.0 - dot(normal, lightDir)), 0.0006);
    float lit = 0.0;
    for (int y = -1; y <= 1; ++y) {
        for (int x = -1; x <= 1; ++x) {
            float depth = texture(uShadowMap, proj.xy + vec2(x, y) * uShadowTexel).r;
            lit += proj.z - bias > depth ? 0.0 : 1.0;
        }
    }
    return lit / 9.0;
}

vec3 shade(vec3 normal, vec3 view, vec3 lightDir, vec3 radiance, vec3 albedo, float gloss) {
    float ndl = max(dot(normal, lightDir), 0.0);
    vec3 halfway = normalize(lightDir + view);
    float spec = pow(max(dot(normal, halfway), 0.0), gloss) * (gloss + 8.0) / 25.0;
    vec3 specularTint = mix(vec3(1.0), albedo, uMetallic);
    return radiance * ndl * (albedo * (1.0 - uMetallic * 0.6) + specularTint * spec);
}

void main() {
    vec3 normal = normalize(vNormal);
    vec3 view = normalize(uCameraPos - vWorldPos);
    float gloss = mix(128.0, 4.0, clamp(uRoughness, 0.0, 1.0));

    vec3 sunDir = normalize(-uSunDirection);
    vec3 color = shade(normal, view, sunDir, uSunColor * uSunIntensity, uAlbedo, gloss) *
                 shadowFactor(normal, sunDir);

    for (int i = 0; i < uPointCount; ++i) {
        vec3 toLight = uPointPosRange[i].xyz - vWorldPos;
        float dist = length(toLight);
        float range = uPointPosRange[i].w;
        if (dist > range) continue;
        float falloff = clamp(1.0 - dist / range, 0.0, 1.0);
        falloff *= falloff;
        color += shade(normal, view, toLight / max(dist, 1e-4),
                       uPointColor[i].rgb * uPointColor[i].a * falloff, uAlbedo, gloss);
    }

    float hemi = normal.y * 0.5 + 0.5;
    color += uAlbedo * mix(uAmbientGround, uAmbientSky, hemi);
    color += uEmissive;

    float fog = 1.0 - exp(-uFogDensity * length(uCameraPos - vWorldPos));
    color = mix(color, uFogColor, clamp(fog, 0.0, 1.0));

    color = color / (color + vec3(1.0));
    fragColor = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}
)";

inline const char* DEPTH_VERTEX = R"(#version 410 core
layout(location = 0) in vec3 aPosition;
layout(location = 3) in mat4 aModel;
uniform mat4 uViewProj;
void main() { gl_Position = uViewProj * aModel * vec4(aPosition, 1.0); }
)";

inline const char* DEPTH_FRAGMENT = R"(#version 410 core
void main() {}
)";

inline const char* LINE_VERTEX = R"(#version 410 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aColor;
uniform mat4 uViewProj;
out vec3 vColor;
void main() {
    vColor = aColor;
    gl_Position = uViewProj * vec4(aPosition, 1.0);
}
)";

inline const char* LINE_FRAGMENT = R"(#version 410 core
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
)";

}  // namespace skein::shaders
