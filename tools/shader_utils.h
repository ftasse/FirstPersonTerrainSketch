#ifndef SHADER_UTILS_H
#define SHADER_UTILS_H

#include <sys/stat.h>
#include <stdio.h>
#include <vector>

#include "GL/glew.h"

GLuint createShaderFromString(GLenum shader_type, const char *str,
                              const char *identifier = "")
{
    GLuint shader = glCreateShader(shader_type);
    glShaderSource(shader, 1, (const char**)&str, NULL);
    glCompileShader(shader);

    GLint status;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint info_log_length;
        glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &info_log_length);
        GLchar *info_log = new GLchar[info_log_length + 1];
        glGetShaderInfoLog(shader, info_log_length, NULL, info_log);
        fprintf(stderr, "Compile failure in %s shader:\n%s\n", identifier, info_log);
        delete[] info_log;
    }

    return shader;
}

GLuint createShader(GLenum shader_type, const char *shader_path)
{
    FILE *shader_file = fopen(shader_path, "r");
    if (!shader_file)
    {
        fprintf(stderr, "Could not open shader file: %s\n", shader_path);
        return 0;
    }

    fseek(shader_file, 0, SEEK_END);
    int size = ftell(shader_file);
    rewind (shader_file);
    char *str = new char [size+1];
    str[size] = '\0';
    fread(str, 1, size, shader_file);

    GLuint shader = createShaderFromString(shader_type, str, shader_path);

    delete [] str;
    return shader;
}

GLuint createProgram(const std::vector<GLuint> &shaders)
{
    GLuint program = glCreateProgram();

    for(size_t i = 0; i < shaders.size(); i++)
        glAttachShader(program, shaders[i]);

    glLinkProgram(program);

    GLint status;
    glGetProgramiv (program, GL_LINK_STATUS, &status);
    if (status == GL_FALSE)
    {
        GLint info_log_length;
        glGetProgramiv(program, GL_INFO_LOG_LENGTH, &info_log_length);

        GLchar *info_log = new GLchar[info_log_length + 1];
        glGetProgramInfoLog(program, info_log_length, NULL, info_log);
        fprintf(stderr, "Linker failure: %s\n", info_log);
        delete[] info_log;
    }

    for(size_t i = 0; i < shaders.size(); i++)
        glDetachShader(program, shaders[i]);

    return program;
}

void deleteProgram(const GLuint &program, const std::vector<GLuint> &shaders)
{
    for(size_t i = 0; i < shaders.size(); i++)
        glDeleteShader(shaders[i]);
    glDeleteProgram(program);
}

#endif // SHADER_UTILS_H
