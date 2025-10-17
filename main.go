package main

import (
	"fmt"
	"image"
	_ "image/jpeg"
	"math"
	"os"
	"runtime"
	"unsafe"

	"github.com/go-gl/gl/v4.1-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
	"github.com/go-gl/mathgl/mgl32"
)

const (
	width  = 800
	height = 600
)

var (
	yaw     float32 = 0
	pitch   float32 = 0
	radius  float32 = 10.0
	lastX   float64
	lastY   float64
	first   = true
)

const vertexShader = `#version 410
in vec2 position;
out vec2 uv;
void main() {
    uv = (position + 1.0) / 2.0;
    gl_Position = vec4(position, 0.0, 1.0);
}`

const fragmentShader = `#version 410
in vec2 uv;
out vec4 color;
uniform sampler2D tex;
uniform mat3 camRot;
uniform vec3 camPos;
void main() {
    vec2 frag = uv * 2.0 - 1.0;
    float aspect = 800.0/600.0;
    vec3 localDir = vec3(frag.x * aspect * tan(radians(45.0)), frag.y * tan(radians(45.0)), -1.0);
    vec3 dir = normalize(camRot * localDir);

    float M = 1.0;
    float rs = 2.0 * M;
    float dtau = 0.005;
    int N = 3000;
    float max_r = 10000.0;

    vec3 pos_cart = camPos;
    float r = length(pos_cart);
    float theta = acos(pos_cart.y / r);
    float phi = atan(pos_cart.x, pos_cart.z);

    vec3 hat_r = pos_cart / r;
    vec3 hat_theta = vec3(cos(theta) * cos(phi), -sin(theta), cos(theta) * sin(phi));
    vec3 hat_phi = vec3(-sin(phi), 0.0, cos(phi));

    float d_r = dot(dir, hat_r);
    float d_th = dot(dir, hat_theta);
    float d_ph = dot(dir, hat_phi);

    float alpha = 1.0 - rs / r;

    float sin_theta = sin(theta);
    if (abs(sin_theta) < 1e-6) {
        d_ph = 0.0;
    }

    float kt = 1.0 / sqrt(alpha);
    float kr = d_r * sqrt(alpha);
    float kth = d_th / r;
    float kph = d_ph / (r * sin_theta);

    vec4 x = vec4(0.0, r, theta, phi);
    vec4 k = vec4(kt, kr, kth, kph);

    bool hit = false;

    for (int i = 0; i < N; i++) {
        float curr_r = x.y;
        if (curr_r < rs) {
            hit = true;
            break;
        }
        if (curr_r > max_r) {
            break;
        }
        float sinth = sin(x.z);
        float costh = cos(x.z);
        float sin2 = sinth * sinth;
        float Mr2 = M / (curr_r * curr_r);
        alpha = 1.0 - rs / curr_r;
        float gamma = 1.0 / alpha;

        vec4 a = vec4(0.0);
        a.x = -2.0 * (Mr2 * gamma) * k.y * k.x;
        a.y = - (Mr2 * alpha) * k.x * k.x + (Mr2 * gamma) * k.y * k.y + curr_r * alpha * k.z * k.z + curr_r * alpha * sin2 * k.w * k.w;
        if (abs(sinth) < 1e-6) {
            a.z = - (2.0 / curr_r) * k.y * k.z;
            a.w = - (2.0 / curr_r) * k.y * k.w;
        } else {
            float cotth = costh / sinth;
            a.z = - (2.0 / curr_r) * k.y * k.z + sinth * costh * k.w * k.w;
            a.w = - (2.0 / curr_r) * k.y * k.w - 2.0 * cotth * k.z * k.w;
        }

        x += dtau * k;
        k += dtau * a;
    }

    if (hit) {
        color = vec4(0.0, 0.0, 0.0, 1.0);
    } else {
        float final_theta = x.z;
        float final_phi = x.w;
        float final_sinth = sin(final_theta);
        float final_costh = cos(final_theta);
        vec3 final_hat_r = vec3(final_sinth * cos(final_phi), final_costh, final_sinth * sin(final_phi));
        vec3 final_hat_theta = vec3(final_costh * cos(final_phi), -final_sinth, final_costh * sin(final_phi));
        vec3 final_hat_phi = vec3(-sin(final_phi), 0.0, cos(final_phi));
        vec3 v = k.y * final_hat_r + x.y * k.z * final_hat_theta + x.y * final_sinth * k.w * final_hat_phi;
        vec3 final_dir = normalize(v);
        float final_theta_samp = acos(final_dir.y);
        float final_phi_samp = atan(final_dir.x, -final_dir.z);
        vec2 sph = vec2((final_phi_samp + 3.14159265) / (2.0 * 3.14159265), 1.0 - final_theta_samp / 3.14159265);
        color = texture(tex, sph);
    }
}`

func main() {
	runtime.LockOSThread()

	if err := glfw.Init(); err != nil {
		panic(err)
	}
	defer glfw.Terminate()

	glfw.WindowHint(glfw.ContextVersionMajor, 4)
	glfw.WindowHint(glfw.ContextVersionMinor, 1)
	glfw.WindowHint(glfw.OpenGLProfile, glfw.OpenGLCoreProfile)
	glfw.WindowHint(glfw.OpenGLForwardCompatible, glfw.True)

	window, err := glfw.CreateWindow(width, height, "Black Hole Sim", nil, nil)
	if err != nil {
		panic(err)
	}
	window.MakeContextCurrent()
	window.SetInputMode(glfw.CursorMode, glfw.CursorDisabled)
	window.SetCursorPosCallback(cursorPosCallback)
	window.SetKeyCallback(keyCallback)
	window.SetScrollCallback(scrollCallback)

	if err := gl.Init(); err != nil {
		panic(err)
	}

	xscale, yscale := window.GetContentScale()
	gl.Viewport(0, 0, int32(float32(width)*xscale), int32(float32(height)*yscale))

	program := createProgram(vertexShader, fragmentShader)

	vertices := []float32{-1, -1, -1, 1, 1, -1, 1, 1}
	vao, vbo := uint32(0), uint32(0)
	gl.GenVertexArrays(1, &vao)
	gl.BindVertexArray(vao)
	gl.GenBuffers(1, &vbo)
	gl.BindBuffer(gl.ARRAY_BUFFER, vbo)
	gl.BufferData(gl.ARRAY_BUFFER, len(vertices)*4, unsafe.Pointer(&vertices[0]), gl.STATIC_DRAW)
	pos := uint32(gl.GetAttribLocation(program, gl.Str("position\x00")))
	gl.EnableVertexAttribArray(pos)
	gl.VertexAttribPointer(pos, 2, gl.FLOAT, false, 0, nil)

	tex := loadTexture("eso0932a.jpg")

	camRotLoc := gl.GetUniformLocation(program, gl.Str("camRot\x00"))
	camPosLoc := gl.GetUniformLocation(program, gl.Str("camPos\x00"))

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT)

		gl.UseProgram(program)
		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, tex)

		sy := float32(math.Sin(float64(yaw)))
		cy := float32(math.Cos(float64(yaw)))
		sp := float32(math.Sin(float64(pitch)))
		cp := float32(math.Cos(float64(pitch)))

		forward := mgl32.Vec3{cy * cp, sp, sy * cp}
		camPos := forward.Mul(-radius)

		right := mgl32.Vec3{-sy, 0, cy}
		up := mgl32.Vec3{-cy * sp, cp, -sy * sp}
		minusForward := forward.Mul(-1)
		rot := mgl32.Mat3FromRows(right, up, minusForward).Transpose()
		gl.UniformMatrix3fv(camRotLoc, 1, false, &rot[0])

		gl.Uniform3f(camPosLoc, camPos[0], camPos[1], camPos[2])

		gl.BindVertexArray(vao)
		gl.DrawArrays(gl.TRIANGLE_STRIP, 0, 4)

		window.SwapBuffers()
		glfw.PollEvents()
	}
}

func cursorPosCallback(w *glfw.Window, x, y float64) {
	if first {
		lastX, lastY = x, y
		first = false
	}
	offsetX := x - lastX
	offsetY := lastY - y
	lastX, lastY = x, y
	sens := 0.02
	yaw += float32(offsetX * sens)
	pitch += float32(offsetY * sens)
	if pitch > 89 {
		pitch = 89
	}
	if pitch < -89 {
		pitch = -89
	}
}

func keyCallback(w *glfw.Window, key glfw.Key, _ int, act glfw.Action, _ glfw.ModifierKey) {
	if act == glfw.Press && key == glfw.KeyEscape {
		w.SetShouldClose(true)
	}
}

func scrollCallback(w *glfw.Window, xoff, yoff float64) {
	radius -= float32(yoff) * 0.5
	if radius < 0.1 {
		radius = 0.1
	}
}

func createProgram(vs, fs string) uint32 {
	vShader := gl.CreateShader(gl.VERTEX_SHADER)
	cstr, free := gl.Strs(vs)
	gl.ShaderSource(vShader, 1, cstr, nil)
	free()
	gl.CompileShader(vShader)
	var vStatus int32
	gl.GetShaderiv(vShader, gl.COMPILE_STATUS, &vStatus)
	if vStatus == gl.FALSE {
		var logLen int32
		gl.GetShaderiv(vShader, gl.INFO_LOG_LENGTH, &logLen)
		log := make([]byte, logLen)
		gl.GetShaderInfoLog(vShader, logLen, nil, &log[0])
		panic(fmt.Sprintf("Vertex shader error: %s", log))
	}

	fShader := gl.CreateShader(gl.FRAGMENT_SHADER)
	cstr, free = gl.Strs(fs)
	gl.ShaderSource(fShader, 1, cstr, nil)
	free()
	gl.CompileShader(fShader)
	var fStatus int32
	gl.GetShaderiv(fShader, gl.COMPILE_STATUS, &fStatus)
	if fStatus == gl.FALSE {
		var logLen int32
		gl.GetShaderiv(fShader, gl.INFO_LOG_LENGTH, &logLen)
		log := make([]byte, logLen)
		gl.GetShaderInfoLog(fShader, logLen, nil, &log[0])
		panic(fmt.Sprintf("Fragment shader error: %s", log))
	}

	prog := gl.CreateProgram()
	gl.AttachShader(prog, vShader)
	gl.AttachShader(prog, fShader)
	gl.LinkProgram(prog)
	var status int32
	gl.GetProgramiv(prog, gl.LINK_STATUS, &status)
	if status == gl.FALSE {
		var logLen int32
		gl.GetProgramiv(prog, gl.INFO_LOG_LENGTH, &logLen)
		log := make([]byte, logLen)
		gl.GetProgramInfoLog(prog, logLen, nil, &log[0])
		panic(fmt.Sprintf("Link error: %s", log))
	}
	return prog
}

func loadTexture(path string) uint32 {
	f, err := os.Open(path)
	if err != nil {
		panic(err)
	}
	img, _, err := image.Decode(f)
	if err != nil {
		panic(err)
	}
	bounds := img.Bounds()
	rgba := image.NewRGBA(bounds)
	for y := 0; y < bounds.Max.Y; y++ {
		yy := bounds.Max.Y - 1 - y
		for x := 0; x < bounds.Max.X; x++ {
			rgba.Set(x, y, img.At(x, yy))
		}
	}
	var tex uint32
	gl.GenTextures(1, &tex)
	gl.BindTexture(gl.TEXTURE_2D, tex)
	gl.TexImage2D(gl.TEXTURE_2D, 0, gl.RGBA, int32(bounds.Max.X), int32(bounds.Max.Y), 0, gl.RGBA, gl.UNSIGNED_BYTE, unsafe.Pointer(&rgba.Pix[0]))
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.REPEAT)
	gl.TexParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE)
	return tex
}