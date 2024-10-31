#include <GL/glut.h>  // Include the GLUT library

// Initialize OpenGL settings
void initialize() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);  // Set background color to dark gray
    glEnable(GL_DEPTH_TEST);                // Enable depth testing for 3D rendering
}

// Render a simple triangle
void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // Clear the screen and depth buffer

    glBegin(GL_TRIANGLES);  // Start drawing a triangle
        glColor3f(1.0f, 0.0f, 0.0f);  // Color of the first vertex (Red)
        glVertex3f(-0.5f, -0.5f, 0.0f);  // First vertex at the bottom-left

        glColor3f(0.0f, 1.0f, 0.0f);  // Color of the second vertex (Green)
        glVertex3f(0.5f, -0.5f, 0.0f);   // Second vertex at the bottom-right

        glColor3f(0.0f, 0.0f, 1.0f);  // Color of the third vertex (Blue)
        glVertex3f(0.0f, 0.5f, 0.0f);  // Third vertex at the top
    glEnd();

    glutSwapBuffers();  // Swap buffers for double buffering
}

// Handle window resizing
void reshape(int width, int height) {
    glViewport(0, 0, width, height);  // Set the viewport to cover the new window
    glMatrixMode(GL_PROJECTION);      // Set projection matrix mode
    glLoadIdentity();                 // Reset projection matrix
    gluPerspective(45.0, (double)width / (double)height, 1.0, 100.0);  // Set perspective
    glMatrixMode(GL_MODELVIEW);       // Return to modelview matrix mode
    glLoadIdentity();                 // Reset modelview matrix
    glTranslatef(0.0f, 0.0f, -2.0f);  // Move back to view the triangle
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);                 // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);  // Enable double buffering
    glutInitWindowSize(640, 480);          // Set the initial window size
    glutCreateWindow("Simple OpenGL Interface");  // Create the window with a title

    initialize();                          // Initialize OpenGL settings

    glutDisplayFunc(display);              // Set the display callback
    glutReshapeFunc(reshape);              // Set the reshape callback

    glutMainLoop();                        // Enter the main loop
    return 0;
}