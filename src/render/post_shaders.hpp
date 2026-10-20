#pragma once

namespace skein::shaders {

inline const char* FULLSCREEN_VERTEX = R"(#version 410 core
out vec2 vUv;
void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
    vUv = p;
    gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

inline const char* SSAO_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out float fragAo;
uniform sampler2D uDepth;
uniform mat4 uProj;
uniform mat4 uInvProj;
uniform vec2 uTexel;
uniform float uRadius;
uniform float uBias;
uniform float uPower;

vec3 viewPos(vec2 uv) {
    float d = texture(uDepth, uv).r;
    vec4 p = uInvProj * vec4(uv * 2.0 - 1.0, d * 2.0 - 1.0, 1.0);
    return p.xyz / p.w;
}

float noise(vec2 p) { return fract(52.9829189 * fract(dot(p, vec2(0.06711056, 0.00583715)))); }

void main() {
    float depth = texture(uDepth, vUv).r;
    if (depth >= 1.0) {
        fragAo = 1.0;
        return;
    }
    vec3 p = viewPos(vUv);
    vec3 px = viewPos(vUv + vec2(uTexel.x, 0.0)) - p;
    vec3 nx = p - viewPos(vUv - vec2(uTexel.x, 0.0));
    vec3 py = viewPos(vUv + vec2(0.0, uTexel.y)) - p;
    vec3 ny = p - viewPos(vUv - vec2(0.0, uTexel.y));
    vec3 dx = abs(px.z) < abs(nx.z) ? px : nx;
    vec3 dy = abs(py.z) < abs(ny.z) ? py : ny;
    vec3 n = normalize(cross(dx, dy));
    float angle = noise(gl_FragCoord.xy) * 6.2831853;
    vec3 r = vec3(cos(angle), sin(angle), 0.0);
    vec3 t = normalize(r - n * dot(r, n));
    vec3 b = cross(n, t);
    float occlusion = 0.0;
    const int SAMPLES = 16;
    for (int i = 0; i < SAMPLES; ++i) {
        float fi = float(i);
        float u = fract(fi * 0.618034 + 0.5 * noise(gl_FragCoord.yx));
        float v = (fi + 0.5) / float(SAMPLES);
        float phi = u * 6.2831853;
        float cosTheta = sqrt(1.0 - v);
        float sinTheta = sqrt(v);
        vec3 dir = vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);
        float scale = mix(0.1, 1.0, (fi / float(SAMPLES)) * (fi / float(SAMPLES)));
        vec3 s = p + (t * dir.x + b * dir.y + n * dir.z) * uRadius * scale;
        vec4 clip = uProj * vec4(s, 1.0);
        vec2 suv = clip.xy / clip.w * 0.5 + 0.5;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;
        float sceneZ = viewPos(suv).z;
        float range = smoothstep(0.0, 1.0, uRadius / max(abs(p.z - sceneZ), 1e-4));
        occlusion += (sceneZ >= s.z + uBias ? 1.0 : 0.0) * range;
    }
    fragAo = pow(clamp(1.0 - occlusion / float(SAMPLES), 0.0, 1.0), uPower);
}
)";

inline const char* SSAO_BLUR_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out float fragAo;
uniform sampler2D uAo;
uniform sampler2D uDepth;
uniform vec2 uStep;
uniform float uProjA;
uniform float uProjB;

float linearDepth(vec2 uv) { return uProjB / (texture(uDepth, uv).r * 2.0 - 1.0 + uProjA); }

void main() {
    float center = linearDepth(vUv);
    float total = 0.0;
    float weight = 0.0;
    for (int i = -4; i <= 4; ++i) {
        vec2 uv = vUv + uStep * float(i);
        float d = linearDepth(uv);
        float w = exp(-float(i * i) / 12.0) * exp(-abs(d - center) / max(center * 0.02, 1e-3));
        total += texture(uAo, uv).r * w;
        weight += w;
    }
    fragAo = total / max(weight, 1e-5);
}
)";

inline const char* BLOOM_PREFILTER_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSource;
uniform vec2 uTexel;
uniform float uThreshold;
uniform float uKnee;
uniform float uExposure;

vec3 prefilter(vec3 c) {
    float bright = max(c.r, max(c.g, c.b));
    float soft = clamp(bright - uThreshold + uKnee, 0.0, 2.0 * uKnee);
    soft = soft * soft / (4.0 * uKnee + 1e-4);
    return c * max(soft, bright - uThreshold) / max(bright, 1e-4);
}

void main() {
    vec3 a = texture(uSource, vUv + uTexel * vec2(-1.0, -1.0)).rgb;
    vec3 b = texture(uSource, vUv + uTexel * vec2(1.0, -1.0)).rgb;
    vec3 c = texture(uSource, vUv + uTexel * vec2(-1.0, 1.0)).rgb;
    vec3 d = texture(uSource, vUv + uTexel * vec2(1.0, 1.0)).rgb;
    vec3 color = prefilter((a + b + c + d) * (0.25 * uExposure));
    fragColor = vec4(min(color, vec3(64.0)), 1.0);
}
)";

inline const char* BLOOM_DOWN_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSource;
uniform vec2 uTexel;
void main() {
    vec2 t = uTexel;
    vec3 a = texture(uSource, vUv + t * vec2(-2.0, 2.0)).rgb;
    vec3 b = texture(uSource, vUv + t * vec2(0.0, 2.0)).rgb;
    vec3 c = texture(uSource, vUv + t * vec2(2.0, 2.0)).rgb;
    vec3 d = texture(uSource, vUv + t * vec2(-2.0, 0.0)).rgb;
    vec3 e = texture(uSource, vUv).rgb;
    vec3 f = texture(uSource, vUv + t * vec2(2.0, 0.0)).rgb;
    vec3 g = texture(uSource, vUv + t * vec2(-2.0, -2.0)).rgb;
    vec3 h = texture(uSource, vUv + t * vec2(0.0, -2.0)).rgb;
    vec3 i = texture(uSource, vUv + t * vec2(2.0, -2.0)).rgb;
    vec3 j = texture(uSource, vUv + t * vec2(-1.0, 1.0)).rgb;
    vec3 k = texture(uSource, vUv + t * vec2(1.0, 1.0)).rgb;
    vec3 l = texture(uSource, vUv + t * vec2(-1.0, -1.0)).rgb;
    vec3 m = texture(uSource, vUv + t * vec2(1.0, -1.0)).rgb;
    vec3 color = e * 0.125 + (a + c + g + i) * 0.03125 + (b + d + f + h) * 0.0625 + (j + k + l + m) * 0.125;
    fragColor = vec4(color, 1.0);
}
)";

inline const char* BLOOM_UP_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSource;
uniform vec2 uTexel;
uniform float uRadius;
void main() {
    vec2 t = uTexel * uRadius;
    vec3 c = texture(uSource, vUv).rgb * 4.0;
    c += (texture(uSource, vUv + vec2(-t.x, 0.0)).rgb + texture(uSource, vUv + vec2(t.x, 0.0)).rgb +
          texture(uSource, vUv + vec2(0.0, -t.y)).rgb + texture(uSource, vUv + vec2(0.0, t.y)).rgb) * 2.0;
    c += texture(uSource, vUv + vec2(-t.x, -t.y)).rgb + texture(uSource, vUv + vec2(t.x, -t.y)).rgb +
         texture(uSource, vUv + vec2(-t.x, t.y)).rgb + texture(uSource, vUv + vec2(t.x, t.y)).rgb;
    fragColor = vec4(c / 16.0, 1.0);
}
)";

inline const char* COMPOSITE_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uHdr;
uniform sampler2D uAo;
uniform sampler2D uBloom;
uniform sampler2D uDepth;
uniform vec2 uTexel;
uniform float uAoAmount;
uniform float uBloomStrength;
uniform float uExposure;
uniform float uSaturation;
uniform float uContrast;
uniform float uVignette;
uniform float uOutline;
uniform float uOutlineWidth;
uniform vec3 uOutlineColor;
uniform float uOutlineLow;
uniform float uOutlineHigh;
uniform float uProjA;
uniform float uProjB;
uniform int uUseAo;
uniform int uUseBloom;

float linearDepth(vec2 uv) { return uProjB / (texture(uDepth, uv).r * 2.0 - 1.0 + uProjA); }

vec3 aces(vec3 x) {
    const mat3 inputMat = mat3(0.59719, 0.07600, 0.02840, 0.35458, 0.90834, 0.13383, 0.04823, 0.01566, 0.83777);
    const mat3 outputMat = mat3(1.60475, -0.10208, -0.00327, -0.53108, 1.10813, -0.07276, -0.07367, -0.00605, 1.07602);
    vec3 v = inputMat * x;
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return clamp(outputMat * (a / b), 0.0, 1.0);
}

void main() {
    vec3 color = texture(uHdr, vUv).rgb;
    if (uUseAo != 0) color *= mix(1.0, texture(uAo, vUv).r, uAoAmount);
    if (uOutline > 0.0) {
        vec2 o = uTexel * uOutlineWidth;
        float d = linearDepth(vUv);
        float l = 1.0 / linearDepth(vUv - vec2(o.x, 0.0));
        float r = 1.0 / linearDepth(vUv + vec2(o.x, 0.0));
        float u = 1.0 / linearDepth(vUv + vec2(0.0, o.y));
        float w = 1.0 / linearDepth(vUv - vec2(0.0, o.y));
        float c = 1.0 / d;
        float curve = (abs(l + r - 2.0 * c) + abs(u + w - 2.0 * c)) * d;
        float ink = min(texture(uHdr, vUv).a, min(min(texture(uHdr, vUv - vec2(o.x, 0.0)).a, texture(uHdr, vUv + vec2(o.x, 0.0)).a),
                                                  min(texture(uHdr, vUv + vec2(0.0, o.y)).a, texture(uHdr, vUv - vec2(0.0, o.y)).a)));
        float edge = smoothstep(uOutlineLow, uOutlineHigh, curve) * clamp(ink, 0.0, 1.0);
        color = mix(color, uOutlineColor, edge * uOutline);
    }
    color *= uExposure;
    if (uUseBloom != 0) color += texture(uBloom, vUv).rgb * uBloomStrength;
    color = aces(color);
    float luma = dot(color, vec3(0.2126, 0.7152, 0.0722));
    color = mix(vec3(luma), color, uSaturation);
    color = clamp((color - 0.5) * uContrast + 0.5, 0.0, 1.0);
    color = pow(color, vec3(1.0 / 2.2));
    vec2 centred = vUv - 0.5;
    color *= 1.0 - uVignette * smoothstep(0.3, 0.9, length(centred) * 1.25);
    fragColor = vec4(color, dot(color, vec3(0.299, 0.587, 0.114)));
}
)";

inline const char* FXAA_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSource;
uniform vec2 uTexel;
void main() {
    const float SPAN_MAX = 8.0;
    const float REDUCE_MUL = 1.0 / 8.0;
    const float REDUCE_MIN = 1.0 / 128.0;
    float lumaNW = texture(uSource, vUv + vec2(-1.0, -1.0) * uTexel).a;
    float lumaNE = texture(uSource, vUv + vec2(1.0, -1.0) * uTexel).a;
    float lumaSW = texture(uSource, vUv + vec2(-1.0, 1.0) * uTexel).a;
    float lumaSE = texture(uSource, vUv + vec2(1.0, 1.0) * uTexel).a;
    vec4 centre = texture(uSource, vUv);
    float lumaM = centre.a;
    float lumaMin = min(lumaM, min(min(lumaNW, lumaNE), min(lumaSW, lumaSE)));
    float lumaMax = max(lumaM, max(max(lumaNW, lumaNE), max(lumaSW, lumaSE)));
    vec2 dir = vec2(-((lumaNW + lumaNE) - (lumaSW + lumaSE)), (lumaNW + lumaSW) - (lumaNE + lumaSE));
    float reduce = max((lumaNW + lumaNE + lumaSW + lumaSE) * 0.25 * REDUCE_MUL, REDUCE_MIN);
    float rcpDirMin = 1.0 / (min(abs(dir.x), abs(dir.y)) + reduce);
    dir = clamp(dir * rcpDirMin, vec2(-SPAN_MAX), vec2(SPAN_MAX)) * uTexel;
    vec3 rgbA = 0.5 * (texture(uSource, vUv + dir * (1.0 / 3.0 - 0.5)).rgb + texture(uSource, vUv + dir * (2.0 / 3.0 - 0.5)).rgb);
    vec3 rgbB = rgbA * 0.5 + 0.25 * (texture(uSource, vUv - dir * 0.5).rgb + texture(uSource, vUv + dir * 0.5).rgb);
    float lumaB = dot(rgbB, vec3(0.299, 0.587, 0.114));
    fragColor = vec4((lumaB < lumaMin || lumaB > lumaMax) ? rgbA : rgbB, 1.0);
}
)";

inline const char* BLIT_FRAGMENT = R"(#version 410 core
in vec2 vUv;
out vec4 fragColor;
uniform sampler2D uSource;
void main() { fragColor = vec4(texture(uSource, vUv).rgb, 1.0); }
)";

}  // namespace skein::shaders
