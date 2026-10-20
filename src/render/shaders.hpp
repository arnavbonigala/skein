#pragma once

namespace skein::shaders {

constexpr int MAX_POINT_LIGHTS = 32;

inline const char* LIT_VERTEX = R"(#version 410 core
layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec2 aUv;
layout(location = 3) in mat4 aModel;
layout(location = 7) in vec4 aColor;

uniform mat4 uViewProj;
uniform mat4 uLightViewProj;
uniform float uTime;
uniform float uWind;

out vec3 vWorldPos;
out vec3 vNormal;
out vec2 vUv;
out vec4 vLightSpace;
out vec3 vTint;

void main() {
    mat4 model = aModel;
    vTint = vec3(model[0][3], model[1][3], model[2][3]) * aColor.rgb;
    model[0][3] = 0.0;
    model[1][3] = 0.0;
    model[2][3] = 0.0;
    vec4 world = model * vec4(aPosition, 1.0);
    if (uWind > 0.0) {
        float sway = max(aPosition.y, 0.0);
        float phase = uTime * 1.9 + world.x * 0.35 + world.z * 0.27;
        world.xz += vec2(sin(phase), cos(phase * 0.8 + 1.3)) * (uWind * sway * sway);
    }
    vWorldPos = world.xyz;
    vNormal = transpose(inverse(mat3(model))) * aNormal;
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
in vec3 vTint;

out vec4 fragColor;

uniform vec3 uCameraPos;
uniform vec3 uAlbedo;
uniform vec3 uEmissive;
uniform float uRoughness;
uniform float uMetallic;
uniform float uInk;

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
uniform int uHdr;
uniform float uToon;
uniform float uRim;
uniform int uSkyGradient;
uniform vec3 uSkyZenith;
uniform vec3 uSkyHorizon;
uniform vec3 uSkyGround;

vec3 skyColor(vec3 dir) {
    if (uSkyGradient == 0) return uFogColor;
    float up = dir.y;
    if (up < 0.0) return mix(uSkyHorizon, uSkyGround, clamp(-up * 4.0, 0.0, 1.0));
    return mix(uSkyHorizon, uSkyZenith, pow(clamp(up, 0.0, 1.0), 0.55));
}

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
    float cel = smoothstep(0.0, 0.1, ndl) * 0.72 + smoothstep(0.5, 0.6, ndl) * 0.28;
    ndl = mix(ndl, cel, uToon);
    vec3 halfway = normalize(lightDir + view);
    float spec = pow(max(dot(normal, halfway), 0.0), gloss) * (gloss + 8.0) / 25.0;
    spec = mix(spec, smoothstep(0.55, 0.6, spec) * 0.5, uToon);
    vec3 specularTint = mix(vec3(1.0), albedo, uMetallic);
    return radiance * ndl * (albedo * (1.0 - uMetallic * 0.6) + specularTint * spec);
}

void main() {
    vec3 albedo = uAlbedo * vTint;
    vec3 normal = normalize(vNormal);
    vec3 view = normalize(uCameraPos - vWorldPos);
    float gloss = mix(128.0, 4.0, clamp(uRoughness, 0.0, 1.0));

    vec3 sunDir = normalize(-uSunDirection);
    float lit = shadowFactor(normal, sunDir);
    lit = mix(lit, smoothstep(0.35, 0.65, lit), uToon);
    vec3 color = shade(normal, view, sunDir, uSunColor * uSunIntensity, albedo, gloss) * lit;

    for (int i = 0; i < uPointCount; ++i) {
        vec3 toLight = uPointPosRange[i].xyz - vWorldPos;
        float dist = length(toLight);
        float range = uPointPosRange[i].w;
        if (dist > range) continue;
        float falloff = clamp(1.0 - dist / range, 0.0, 1.0);
        falloff *= falloff;
        color += shade(normal, view, toLight / max(dist, 1e-4),
                       uPointColor[i].rgb * uPointColor[i].a * falloff, albedo, gloss);
    }

    float hemi = normal.y * 0.5 + 0.5;
    color += albedo * mix(uAmbientGround, uAmbientSky, hemi);
    float facing = 1.0 - max(dot(normal, view), 0.0);
    float rim = smoothstep(0.55, 0.75, facing) * (0.35 + 0.65 * lit * max(dot(normal, sunDir) + 0.4, 0.0))
        * clamp(1.0 - normal.y, 0.0, 1.0);
    color += uRim * rim * mix(albedo, vec3(1.0), 0.5) * uSunColor;
    color += uEmissive;

    float fog = 1.0 - exp(-uFogDensity * length(uCameraPos - vWorldPos));
    color = mix(color, skyColor(-view), clamp(fog, 0.0, 1.0));

    if (uHdr != 0) {
        fragColor = vec4(color, uInk);
        return;
    }
    color = color / (color + vec3(1.0));
    fragColor = vec4(pow(color, vec3(1.0 / 2.2)), 1.0);
}
)";

inline const char* SKY_VERTEX = R"(#version 410 core
out vec2 vNdc;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vNdc = p;
    gl_Position = vec4(p, 1.0, 1.0);
}
)";

inline const char* SKY_FRAGMENT = R"(#version 410 core
in vec2 vNdc;
out vec4 fragColor;
uniform mat4 uInvViewProj;
uniform vec3 uCameraPos;
uniform vec3 uSkyZenith;
uniform vec3 uSkyHorizon;
uniform vec3 uSkyGround;
uniform vec3 uSunDirection;
uniform vec3 uSunColor;
uniform float uSunSize;
uniform int uHdr;

void main() {
    vec4 far = uInvViewProj * vec4(vNdc, 1.0, 1.0);
    vec3 dir = normalize(far.xyz / far.w - uCameraPos);
    vec3 color;
    if (dir.y < 0.0) color = mix(uSkyHorizon, uSkyGround, clamp(-dir.y * 4.0, 0.0, 1.0));
    else color = mix(uSkyHorizon, uSkyZenith, pow(clamp(dir.y, 0.0, 1.0), 0.55));
    float d = dot(dir, normalize(-uSunDirection));
    float disc = smoothstep(cos(uSunSize), cos(uSunSize * 0.85), d);
    float glow = pow(max(d, 0.0), 64.0) * 0.6 + pow(max(d, 0.0), 8.0) * 0.12;
    color += uSunColor * (disc * 6.0 + glow);
    if (uHdr == 0) color = pow(color / (color + vec3(1.0)), vec3(1.0 / 2.2));
    fragColor = vec4(color, 1.0);
}
)";

inline const char* DEPTH_VERTEX = R"(#version 410 core
layout(location = 0) in vec3 aPosition;
layout(location = 3) in mat4 aModel;
uniform mat4 uViewProj;
void main() {
    mat4 model = aModel;
    model[0][3] = 0.0;
    model[1][3] = 0.0;
    model[2][3] = 0.0;
    gl_Position = uViewProj * model * vec4(aPosition, 1.0);
}
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
