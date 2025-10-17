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
	yaw   float32 = 0
	pitch float32 = 0
	lastX float64
	lastY float64
	first = true
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
void main() {
    vec2 frag = uv * 2.0 - 1.0;
    float aspect = 800.0/600.0;
    vec3 dir = normalize(vec3(frag.x * aspect * tan(radians(45.0)), frag.y * tan(radians(45.0)), -1.0));
    dir = camRot * dir;
    float theta = acos(dir.y);
    float phi = atan(dir.x, -dir.z);
    vec2 sph = vec2((phi + 3.14159265) / (2.0 * 3.14159265), 1.0 - theta / 3.14159265);
    color = texture(tex, sph);
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

	camLoc := gl.GetUniformLocation(program, gl.Str("camRot\x00"))

	for !window.ShouldClose() {
		gl.Clear(gl.COLOR_BUFFER_BIT)

		gl.UseProgram(program)
		gl.ActiveTexture(gl.TEXTURE0)
		gl.BindTexture(gl.TEXTURE_2D, tex)
		sy := float32(math.Sin(float64(yaw)))
		cy := float32(math.Cos(float64(yaw)))
		sp := float32(math.Sin(float64(pitch)))
		cp := float32(math.Cos(float64(pitch)))
		right := mgl32.Vec3{-sy, 0, cy}
		up := mgl32.Vec3{-cy * sp, cp, -sy * sp}
		minus_forward := mgl32.Vec3{-cy * cp, -sp, -sy * cp}
		rot := mgl32.Mat3FromRows(right, up, minus_forward).Transpose()
		gl.UniformMatrix3fv(camLoc, 1, false, &rot[0])

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

func createProgram(vs, fs string) uint32 {
	vShader := gl.CreateShader(gl.VERTEX_SHADER)
	cstr, free := gl.Strs(vs)
	gl.ShaderSource(vShader, 1, cstr, nil)
	free()
	gl.CompileShader(vShader)
	fShader := gl.CreateShader(gl.FRAGMENT_SHADER)
	cstr, free = gl.Strs(fs)
	gl.ShaderSource(fShader, 1, cstr, nil)
	free()
	gl.CompileShader(fShader)
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