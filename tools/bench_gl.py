#!/usr/bin/env python3
"""Same-scene OpenGL benchmark for native Windows vs WSL (WSLg).

Renders assets/bench_scene.npy (robot + gravity dam) with vsync off:
  onscreen  : draw + swap to a 1280x720 window (includes the desktop/compositor path)
  offscreen : draw into a framebuffer + glFinish (pure GPU work)
for 1 and 8 copies of the scene.
usage: python bench_gl.py <bench_scene.npy> [frames]
needs: pip install moderngl glfw numpy"""
import sys
import time

import glfw
import moderngl
import numpy as np

VS = '''#version 330
uniform mat4 mvp; uniform vec3 off;
in vec3 in_pos; in vec3 in_nrm; out vec3 n;
void main() { n = in_nrm; gl_Position = mvp * vec4(in_pos + off, 1.0); }'''
FS = '''#version 330
in vec3 n; out vec4 color;
void main() { float l = 0.35 + 0.65 * abs(dot(normalize(n), normalize(vec3(-0.35, -0.6, 0.72))));
              color = vec4(vec3(0.75, 0.73, 0.68) * l, 1.0); }'''


def look_at(eye, target, fov=1.0, aspect=16 / 9, near=0.1, far=300.0):
    f = target - eye; f /= np.linalg.norm(f)
    s = np.cross(f, [0, 0, 1.0]); s /= np.linalg.norm(s)
    u = np.cross(s, f)
    view = np.eye(4); view[0, :3], view[1, :3], view[2, :3] = s, u, -f
    view[:3, 3] = -view[:3, :3] @ eye
    t = 1 / np.tan(fov / 2)
    proj = np.zeros((4, 4)); proj[0, 0] = t / aspect; proj[1, 1] = t
    proj[2, 2] = (far + near) / (near - far); proj[2, 3] = 2 * far * near / (near - far); proj[3, 2] = -1
    return (proj @ view).astype('f4')


def main():
    data = np.load(sys.argv[1]).astype('f4')
    frames = int(sys.argv[2]) if len(sys.argv) > 2 else 300
    glfw.init()
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3); glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    win = glfw.create_window(1280, 720, 'wcr bench', None, None)
    glfw.make_context_current(win)
    glfw.swap_interval(0)
    ctx = moderngl.create_context()
    ctx.enable(moderngl.DEPTH_TEST)
    print(f"renderer: {ctx.info['GL_RENDERER']} | {len(data) // 3} triangles per scene copy", flush=True)
    prog = ctx.program(vertex_shader=VS, fragment_shader=FS)
    vao = ctx.vertex_array(prog, [(ctx.buffer(data.tobytes()), '3f 3f', 'in_pos', 'in_nrm')])
    fbo = ctx.simple_framebuffer((1280, 720))
    results = {}
    for mode in ('onscreen', 'offscreen'):
        for copies in (1, 8):
            target = ctx.screen if mode == 'onscreen' else fbo
            target.use()
            for warm in range(10):
                ctx.clear(0.6, 0.7, 0.85); vao.render()
            ctx.finish()
            t0 = time.perf_counter()
            for i in range(frames):
                a = 2 * np.pi * i / frames
                eye = np.array([30 * np.cos(a), 30 * np.sin(a) - 5, 12.0])
                prog['mvp'].write(look_at(eye, np.array([0, 4.0, 5.0])).T.tobytes())
                ctx.clear(0.6, 0.7, 0.85)
                for c in range(copies):
                    prog['off'].value = (0.0, 0.0, 0.0) if c == 0 else (float(c * 35), 0.0, 0.0)
                    vao.render()
                if mode == 'onscreen':
                    glfw.swap_buffers(win); glfw.poll_events()
            ctx.finish()
            dt = time.perf_counter() - t0
            results[(mode, copies)] = frames / dt
            print(f'{mode:9s} x{copies}: {frames / dt:7.1f} fps  ({1000 * dt / frames:6.2f} ms/frame)', flush=True)
    glfw.terminate()


if __name__ == '__main__':
    main()
