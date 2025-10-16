package main

import (
	"github.com/go-gl/gl/all-core/gl"
	"github.com/go-gl/glfw/v3.3/glfw"
)

func main() {
	// Initialize GLFW
    if err := glfw.Init(); err != nil {
        panic(err)
    }
    defer glfw.Terminate()

    // Create a window
    window, err := glfw.CreateWindow(800, 600, "Black Hole Simulation", nil, nil)
    if err != nil {
        panic(err)
    }

    // Make the window's context current
    window.MakeContextCurrent()

    // Initialize OpenGL
    if err := gl.Init(); err != nil {
        panic(err)
    }

    // Main loop
    for !window.ShouldClose() {
        // Handle events
        glfw.PollEvents()

        // Render a simple triangle
        gl.Clear(gl.COLOR_BUFFER_BIT)
        gl.Begin(gl.TRIANGLES)
        gl.Color3f(1, 0, 0)
        gl.Vertex2f(-0.5, -0.5)
        gl.Vertex2f(0.5, -0.5)
        gl.Vertex2f(0, 0.5)
        gl.End()

        // Swap buffers
        window.SwapBuffers()
    }
}