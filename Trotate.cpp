#include <GL/glut.h>  // Include the GLUT library
#include <iostream>
#include <cmath> // For sine and cosine functions

class Triangle{
private:
    float position[3][3];
    float color[3][3];

public:
    Triangle(float *x,float *y,float *z){
        for (int i=0;i<3;i++){
            //Position
            position[0][i]=x[i];
            position[1][i]=y[i];
            position[2][i]=z[i];
            //Color
            color[0][i]=0.0f;
            color[1][i]=0.0f;
            color[2][i]=0.0f;
        }
    };

    void changecol(float *cx,float *cy,float *cz){
        for (int i=0;i<3;i++){
            color[0][i]=cx[i];
            color[1][i]=cy[i];
            color[2][i]=cz[i];
        };
    };

    // Display the triangle
    void display() {
        glBegin(GL_TRIANGLES);  // Start drawing a triangle
        for (int i = 0; i < 3; i++) {
            glColor3f(color[i][0], color[i][1], color[i][2]);  // Color of the i-th vertex 
            glVertex3f(position[i][0], position[i][1], position[i][2]);  // i-th vertex position
        }
        glEnd();
    };

    };

// Global variable for the triangle
Triangle *triangle;

// Function to initialize OpenGL settings
void initialize() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // Set background color to dark gray
    glEnable(GL_DEPTH_TEST);                // Enable depth testing for 3D rendering
}

// Function to handle window resizing
void reshape(int width, int height) {
    glViewport(0, 0, width, height);  // Set the viewport to cover the new window
    glMatrixMode(GL_PROJECTION);      // Set projection matrix mode
    glLoadIdentity();                 // Reset projection matrix
    gluPerspective(45.0, (double)width / (double)height, 1.0, 100.0);  // Set perspective
    glMatrixMode(GL_MODELVIEW);       // Return to modelview matrix mode
    glLoadIdentity();                 // Reset modelview matrix
    glTranslatef(0.0f, 0.0f, -2.0f);  // Move back to view the triangle
}

// Timer function to change colors
void timer(int value) {
    static float hue = 0.0f;  // Start from 0.0
    float colorX[3] = { std::sin(hue), std::sin(hue + 2.0f * 3.14159f / 3.0f), std::sin(hue + 4.0f * 3.14159f / 3.0f) }; // Vertex 0
    float colorY[3] = { std::sin(hue + 2.0f), std::sin(hue + 2.0f + 2.0f * 3.14159f / 3.0f), std::sin(hue + 2.0f + 4.0f * 3.14159f / 3.0f) }; // Vertex 1
    float colorZ[3] = { std::sin(hue + 4.0f), std::sin(hue + 4.0f + 2.0f * 3.14159f / 3.0f), std::sin(hue + 4.0f + 4.0f * 3.14159f / 3.0f) }; // Vertex 2

    // Update colors
    triangle->changecol(colorX, colorY, colorZ);

    hue += 0.1f;  // Increment hue for next color change
    if (hue > 2.0f * 3.14159f) { // Keep hue within 0 to 2π
        hue = 0.0f;
    }

    glutPostRedisplay();  // Request a redraw
    glutTimerFunc(500, timer, 0); // Call this function again after 500 ms
}

// Function for rendering the scene
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the buffers
    triangle->display(); // Render the triangle
    glutSwapBuffers();   // Swap the front and back buffers
}

// Main function
int main(int argc, char** argv) {
    // Initialize GLUT
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // Set display mode
    glutInitWindowSize(800, 600); // Set window size
    glutCreateWindow("OpenGL Triangle"); // Create window

    // Initialize OpenGL
    initialize();

    // Define the triangle vertices
    float x[3] = { -0.5f, 0.5f, 0.0f }; // X-coordinates
    float y[3] = { -0.5f, -0.5f, 0.5f }; // Y-coordinates
    float z[3] = { 0.0f, 0.0f, 0.0f };   // Z-coordinates

    // Create the triangle
    triangle = new Triangle(x, y, z);

    // Change the initial colors of the triangle vertices (example: red, green, blue)
    float initialColorX[3] = { 1.0f, 0.0f, 0.0f }; // Initial color for vertex 0
    float initialColorY[3] = { 0.0f, 1.0f, 0.0f }; // Initial color for vertex 1
    float initialColorZ[3] = { 0.0f, 0.0f, 1.0f }; // Initial color for vertex 2
    triangle->changecol(initialColorX, initialColorY, initialColorZ);

    // Register callback functions
    glutDisplayFunc(display); // Set the display function
    glutReshapeFunc(reshape); // Set the reshape function

    // Start the timer
    glutTimerFunc(1, timer, 0); // Call the timer function after 500 ms

    // Enter the GLUT main loop
    glutMainLoop();

    // Clean up (this code won't be reached)
    delete triangle; // Free the triangle object
    return 0;
}