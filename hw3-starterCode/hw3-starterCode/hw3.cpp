/* **************************
 * CSCI 420
 * Assignment 3 Raytracer
 * Name: Airi Pham
 * *************************
*/

#ifdef WIN32
#include <windows.h>
#endif

#if defined(WIN32) || defined(linux)
#include <GL/gl.h>
#include <GL/glut.h>
#elif defined(__APPLE__)
#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef WIN32
#define strcasecmp _stricmp
#endif

#include <imageIO.h>
#include <glm/glm.hpp>
#include <cmath>
#include <limits>
#include <algorithm>
#include <iostream>

#define MAX_TRIANGLES 20000
#define MAX_SPHERES 100
#define MAX_LIGHTS 100

char* filename = NULL;

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2

int mode = MODE_DISPLAY;

//you may want to make these smaller for debugging purposes
#define WIDTH 640
#define HEIGHT 480

//the field of view of the camera
#define fov 60.0
#define PI 3.14159
#define eps 0.00001

unsigned char buffer[HEIGHT][WIDTH][3];
double min_x, min_y;
double c_width, c_height;

struct Vertex
{
	glm::dvec3 position;
	glm::dvec3 color_diffuse;
	glm::dvec3 color_specular;
	glm::dvec3 normal;
	double shininess;
};

struct Triangle
{
	Vertex v[3];
};

struct Sphere
{
	glm::dvec3 position;
	glm::dvec3 color_diffuse;
	glm::dvec3 color_specular;
	double shininess;
	double radius;
};

struct Light
{
	glm::dvec3 position;
	glm::dvec3 color;
};

struct Ray
{
	glm::dvec3 src;
	glm::dvec3 dir;
};

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
glm::dvec3 ambient_light;
glm::dvec3 black_col = glm::dvec3(0.0, 0.0, 0.0);
glm::dvec3 cam_pos = glm::dvec3(0.0, 0.0, 0.0);
int num_triangles = 0;
int num_spheres = 0;
int num_lights = 0;

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b);
void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b);

bool intersect_triangle(Ray& ray, double& hit_dist, int& tri_idx, double& u, double& v)
{
	// get intersection
	// test point inside triangle: project 3D to 2D and use baycentric coordinate in 2D
	hit_dist = std::numeric_limits<double>::max();
	tri_idx = -1;
	for (int i = 0; i < num_triangles; i++)
	{
		Triangle tri = triangles[i];
		glm::dvec3 pos_a = (tri.v[0].position);
		glm::dvec3 pos_b = (tri.v[1].position);
		glm::dvec3 pos_c = (tri.v[2].position);
		glm::dvec3 ab = pos_b - pos_a;
		glm::dvec3 bc = pos_c - pos_b;
		glm::dvec3 ca = pos_a - pos_c;
		glm::dvec3 normal = glm::normalize(glm::cross(ab, ca));
		double check_paral = glm::dot(normal, ray.dir);
		if (abs(check_paral) < eps) // ray parallel to plane
		{
			std::cout << "Parallel with ray" << std::endl;
			continue;
		}
		// find intersectional point
		double coeff_d = -glm::dot(normal, pos_a);
		double t = -(glm::dot(normal, ray.src) + coeff_d) / (check_paral);
		if (t < eps)
		{
			std::cout << "Too small dist : " << t << std::endl;
			continue;
		}
		glm::dvec3 intersect = ray.src + ray.dir * t;
		// test inside-outside test: barycentric interpolation
		//glm::dvec3 xy_plane = glm::dvec3(1.0, 1.0, 0.0);
		//glm::dvec3 yz_plane = glm::dvec3(0.0, 1.0, 1.0);
		//glm::dvec3 zx_plane = glm::dvec3(1.0, 0.0, 1.0);
		//double proj_xy = glm::dot(xy_plane, normal);
		//double tri_area, pbc_area, pca_area, pab_area;
		//if (glm::dot(xy_plane, normal) != 0.0) // project on xy plane
		//{
		//	tri_area = 0.5 * ((pos_b.x - pos_a.x) * (pos_c.y - pos_a.y) - (pos_c.x - pos_a.x) * (pos_b.y - pos_a.y));
		//	pbc_area = 0.5 * ((pos_b.x - intersect.x) * (pos_c.y - intersect.y) - (pos_c.x - intersect.x) * (pos_b.y - intersect.y));
		//	pca_area = 0.5 * ((pos_a.x - intersect.x) * (pos_c.y - intersect.y) - (pos_c.x - intersect.x) * (pos_a.y - intersect.y));
		//	pab_area = 0.5 * ((pos_b.x - intersect.x) * (pos_a.y - intersect.y) - (pos_a.x - intersect.x) * (pos_b.y - intersect.y));
		//}
		//else if (glm::dot(yz_plane, normal) != 0.0) // project on yz plane
		//{
		//	tri_area = 0.5 * ((pos_b.y - pos_a.y) * (pos_c.z - pos_a.z) - (pos_c.y - pos_a.y) * (pos_b.z - pos_a.z));
		//	pbc_area = 0.5 * ((pos_b.y - intersect.y) * (pos_c.z - intersect.z) - (pos_c.y - intersect.y) * (pos_b.z - intersect.z));
		//	pca_area = 0.5 * ((pos_a.y - intersect.y) * (pos_c.z - intersect.z) - (pos_c.y - intersect.y) * (pos_a.z - intersect.z));
		//	pab_area = 0.5 * ((pos_b.y - intersect.y) * (pos_a.z - intersect.z) - (pos_a.y - intersect.y) * (pos_b.z - intersect.z));
		//}
		//else if (glm::dot(zx_plane, normal) != 0.0) // project on zx plane
		//{
		//	tri_area = 0.5 * ((pos_b.z - pos_a.z) * (pos_c.x - pos_a.x) - (pos_c.z - pos_a.z) * (pos_b.x - pos_a.x));
		//	pbc_area = 0.5 * ((pos_b.z - intersect.z) * (pos_c.x - intersect.x) - (pos_c.z - intersect.z) * (pos_b.x - intersect.x));
		//	pca_area = 0.5 * ((pos_a.z - intersect.z) * (pos_c.x - intersect.x) - (pos_c.z - intersect.z) * (pos_a.x - intersect.x));
		//	pab_area = 0.5 * ((pos_b.z - intersect.z) * (pos_a.x - intersect.x) - (pos_a.z - intersect.z) * (pos_b.x - intersect.x));
		//}
		//if (tri_area == 0.0)
		//	continue;

		//double alpha = pbc_area / tri_area;
		//double beta = pca_area / tri_area;
		double sqr_normal = glm::dot(normal, normal);
		if (abs(sqr_normal) < eps) continue;
		double alpha = glm::dot(normal, glm::cross(ab, intersect - pos_a)) / sqr_normal;
		double beta = glm::dot(normal, glm::cross(bc, intersect - pos_b)) / sqr_normal;
		double gamma = glm::dot(normal, glm::cross(ca, intersect - pos_c)) / sqr_normal;
		// outside of triangle
		if (alpha < 0.0 || beta < 0.0 || gamma < 0.0) continue; // outside of triangle
		else if (t < hit_dist && t > eps)
		{
			hit_dist = t;
			tri_idx = i;
			u = alpha;
			v = beta;
		}
	}
	return tri_idx > -1;
}

bool intersect_sphere(Ray& ray, double& hit_dist, int& sph_idx)
{
	hit_dist = std::numeric_limits<double>::max();
	sph_idx = -1;
	// iterate to find the closest hit point among the spheres
	for (int i = 0; i < num_spheres; i++)
	{
		Sphere sph = spheres[i];
		glm::vec3 dist = ray.src - sph.position;
		double coeff_a = pow(ray.dir.x, 2) + pow(ray.dir.y, 2) + pow(ray.dir.z, 2);
		double coeff_b = 2 * (ray.dir.x * dist.x + ray.dir.y * dist.y + ray.dir.z * dist.z);
		double coeff_c = glm::dot(dist, dist) - pow(sph.radius, 2);
		double delta = pow(coeff_b, 2) - 4 * coeff_c;
		if (delta < 0.0) // ignore
			continue;
		double t0 = (-coeff_b + sqrt(delta)) / 2.0;
		double t1 = (-coeff_b - sqrt(delta)) / 2.0;
		double result = std::min(t0, t1);
		if (result > eps && result < hit_dist)
		{
			hit_dist = result;
			sph_idx = i;
		}
	}
	return sph_idx > -1;
}

glm::dvec3 calc_phong(const Vertex& hit_point, const Light& light)
{
	glm::dvec3 n = hit_point.normal;
	glm::dvec3 l = glm::normalize(light.position - hit_point.position);
	glm::dvec3 v = glm::normalize(hit_point.position - cam_pos);
	glm::dvec3 r = 2.0 * glm::dot(l, n) * n - l;
	double diffuse_coeff = std::max(glm::dot(l, n), 0.0);
	double specular_coeff = pow(std::max(glm::dot(r, v), 0.0), hit_point.shininess);
	glm::dvec3 I = light.color * (diffuse_coeff * hit_point.color_diffuse + specular_coeff * hit_point.color_specular);
	return I;
}

bool is_in_shadow(const Light& light, const Vertex& hit_point)
{
	double hit_dist_sph, hit_dist_tri, u, v;
	int sph_idx, tri_idx;
	glm::dvec3 light_pos = (light.position);
	glm::dvec3 hit_pos = (hit_point.position);
	glm::dvec3 shadow_dir = glm::normalize(light_pos - hit_pos);
	Ray shadow_ray = { hit_pos, shadow_dir };
	bool hit_triangle = intersect_triangle(shadow_ray, hit_dist_tri, tri_idx, u, v);
	bool hit_sphere = intersect_sphere(shadow_ray, hit_dist_sph, sph_idx);
	// if not in shadow
	if (!hit_triangle && !hit_sphere)
	{
		return false;
	}
	//// if in shadow
	//glm::dvec3 shadow_hit_pos;
	//if ((hit_sphere && !hit_triangle)
	//	|| (hit_sphere && hit_triangle && hit_dist_sph < hit_dist_tri))
	//{
	//	shadow_hit_pos = shadow_ray.src + shadow_ray.dir * hit_dist_sph;
	//}
	//else
	//{
	//	shadow_hit_pos = shadow_ray.src + shadow_ray.dir * hit_dist_tri;
	//}
	/*if (glm::length(hit_pos - light_pos) - glm::length(hit_pos - shadow_hit_pos) > eps)
		return false;*/
	return true;
}

glm::dvec3 calc_color(Ray& ray)
{
	double hit_dist_sph, hit_dist_tri;
	int sph_idx, tri_idx;
	double u, v;
	bool hit_sphere = intersect_sphere(ray, hit_dist_sph, sph_idx);
	bool hit_triangle = intersect_triangle(ray, hit_dist_tri, tri_idx, u, v);
	Vertex hit;
	Sphere intersected_sph;
	Triangle intersected_tri;
	if (!hit_sphere && !hit_triangle)
		return black_col;
	else if ((hit_sphere && !hit_triangle)
		|| (hit_sphere && hit_triangle && hit_dist_sph < hit_dist_tri)) // hit sphere first
	{
		intersected_sph = spheres[sph_idx];
		glm::dvec3 sph_pos = glm::dvec3(intersected_sph.position[0],
			intersected_sph.position[1],
			intersected_sph.position[2]);
		hit.position = ray.src + ray.dir * hit_dist_sph;
		hit.normal = glm::normalize(hit.position - sph_pos);
		// std::cout << "Sph Normal: " << hit.normal.x << " " << hit.normal.y << " " << hit.normal.z << std::endl;
		hit.color_diffuse = intersected_sph.color_diffuse;
		hit.color_specular = intersected_sph.color_specular;
		hit.shininess = intersected_sph.shininess;
	}
	else
	{
		std::cout << "Hit a triangle" << std::endl;
		intersected_tri = triangles[tri_idx];
		Vertex point_a = intersected_tri.v[0];
		Vertex point_b = intersected_tri.v[1];
		Vertex point_c = intersected_tri.v[2];
		hit.position = ray.src + ray.dir * hit_dist_tri;
		hit.normal = glm::normalize(point_a.normal * u + point_b.normal * v + point_c.normal * (1.0 - u - v));
		hit.color_diffuse = point_a.color_diffuse * u + point_b.color_diffuse * v + point_c.color_diffuse * (1.0 - u - v);
		hit.color_specular = point_a.color_specular * u + point_b.color_specular * v + point_c.color_specular * (1.0 - u - v);
		hit.shininess = point_a.shininess * u + point_b.shininess * v + point_c.shininess * (1.0 - u - v);
	}

	glm::dvec3 color = black_col;
	for (int i = 0; i < num_lights; i++) // accumulate all light
	{
		if (!is_in_shadow(lights[i], hit))
		{
			color = color + calc_phong(hit, lights[i]);
		}
	}
	return color;
}

//MODIFY THIS FUNCTION
void init_scene()
{
	double asp = (double)WIDTH / (double)HEIGHT;
	double rad_fov = fov * PI / 180.0;
	double y = tan(rad_fov / 2.0);
	min_x = -asp * y;
	min_y = -y;
	c_width = (2.0 * asp * y) / WIDTH;
	c_height = (2.0 * y) / HEIGHT;
}


void draw_scene()
{
	init_scene();
	//for loop sending rays to every point within the image
	for (unsigned int x = 0; x < WIDTH; x++)
	{
		glPointSize(2.0);
		glBegin(GL_POINTS);
		for (unsigned int y = 0; y < HEIGHT; y++)
		{
			// init rays
			Ray ray;
			ray.src = cam_pos;
			double pixel_x = min_x + x * c_width + c_width / 2.0;
			double pixel_y = min_y + y * c_height + c_height / 2.0;
			ray.dir = glm::normalize(glm::dvec3(pixel_x, pixel_y, -1.0));
			glm::dvec3 col = calc_color(ray) + ambient_light;
			plot_pixel(x, y,
				static_cast<int>(col.x * 255),
				static_cast<int>(col.y * 255),
				static_cast<int>(col.z * 255));
		}
		glEnd();
		glFlush();
	}
	printf("Done!\n"); fflush(stdout);
}

void plot_pixel_display(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
	glColor3f(((float)r) / 255.0f, ((float)g) / 255.0f, ((float)b) / 255.0f);
	glVertex2i(x, y);
}

void plot_pixel_jpeg(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
	buffer[y][x][0] = r;
	buffer[y][x][1] = g;
	buffer[y][x][2] = b;
}

void plot_pixel(int x, int y, unsigned char r, unsigned char g, unsigned char b)
{
	plot_pixel_display(x, y, r, g, b);
	if (mode == MODE_JPEG)
		plot_pixel_jpeg(x, y, r, g, b);
}

void save_jpg()
{
	printf("Saving JPEG file: %s\n", filename);

	ImageIO img(WIDTH, HEIGHT, 3, &buffer[0][0][0]);
	if (img.save(filename, ImageIO::FORMAT_JPEG) != ImageIO::OK)
		printf("Error in Saving\n");
	else
		printf("File saved Successfully\n");
}

void parse_check(const char* expected, char* found)
{
	if (strcasecmp(expected, found))
	{
		printf("Expected '%s ' found '%s '\n", expected, found);
		printf("Parse error, abnormal abortion\n");
		exit(0);
	}
}

void parse_doubles(FILE* file, const char* check, glm::dvec3& p)
{
	char str[100];
	fscanf(file, "%s", str);
	parse_check(check, str);
	fscanf(file, "%lf %lf %lf", &p.x, &p.y, &p.z);
	printf("%s %lf %lf %lf\n", check, p.x, p.y, p.z);
}

void parse_rad(FILE* file, double* r)
{
	char str[100];
	fscanf(file, "%s", str);
	parse_check("rad:", str);
	fscanf(file, "%lf", r);
	printf("rad: %f\n", *r);
}

void parse_shi(FILE* file, double* shi)
{
	char s[100];
	fscanf(file, "%s", s);
	parse_check("shi:", s);
	fscanf(file, "%lf", shi);
	printf("shi: %f\n", *shi);
}

int loadScene(char* argv)
{
	FILE* file = fopen(argv, "r");
	int number_of_objects;
	char type[50];
	Triangle t;
	Sphere s;
	Light l;
	fscanf(file, "%i", &number_of_objects);

	printf("number of objects: %i\n", number_of_objects);

	parse_doubles(file, "amb:", ambient_light);

	for (int i = 0; i < number_of_objects; i++)
	{
		fscanf(file, "%s\n", type);
		printf("%s\n", type);
		if (strcasecmp(type, "triangle") == 0)
		{
			printf("found triangle\n");
			for (int j = 0; j < 3; j++)
			{
				parse_doubles(file, "pos:", t.v[j].position);
				parse_doubles(file, "nor:", t.v[j].normal);
				parse_doubles(file, "dif:", t.v[j].color_diffuse);
				parse_doubles(file, "spe:", t.v[j].color_specular);
				parse_shi(file, &t.v[j].shininess);
			}

			if (num_triangles == MAX_TRIANGLES)
			{
				printf("too many triangles, you should increase MAX_TRIANGLES!\n");
				exit(0);
			}
			triangles[num_triangles++] = t;
		}
		else if (strcasecmp(type, "sphere") == 0)
		{
			printf("found sphere\n");

			parse_doubles(file, "pos:", s.position);
			parse_rad(file, &s.radius);
			parse_doubles(file, "dif:", s.color_diffuse);
			parse_doubles(file, "spe:", s.color_specular);
			parse_shi(file, &s.shininess);

			if (num_spheres == MAX_SPHERES)
			{
				printf("too many spheres, you should increase MAX_SPHERES!\n");
				exit(0);
			}
			spheres[num_spheres++] = s;
		}
		else if (strcasecmp(type, "light") == 0)
		{
			printf("found light\n");
			parse_doubles(file, "pos:", l.position);
			parse_doubles(file, "col:", l.color);

			if (num_lights == MAX_LIGHTS)
			{
				printf("too many lights, you should increase MAX_LIGHTS!\n");
				exit(0);
			}
			lights[num_lights++] = l;
		}
		else
		{
			printf("unknown type in scene description:\n%s\n", type);
			exit(0);
		}
	}
	return 0;
}

void display()
{
}

void init()
{
	glMatrixMode(GL_PROJECTION);
	glOrtho(0, WIDTH, 0, HEIGHT, 1, -1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
{
	//hack to make it only draw once
	static int once = 0;
	if (!once)
	{
		draw_scene();
		if (mode == MODE_JPEG)
			save_jpg();
	}
	once = 1;
}

int main(int argc, char** argv)
{
	if ((argc < 2) || (argc > 3))
	{
		printf("Usage: %s <input scenefile> [output jpegname]\n", argv[0]);
		exit(0);
	}
	if (argc == 3)
	{
		mode = MODE_JPEG;
		filename = argv[2];
	}
	else if (argc == 2)
		mode = MODE_DISPLAY;

	glutInit(&argc, argv);
	loadScene(argv[1]);

	glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
	glutInitWindowPosition(0, 0);
	glutInitWindowSize(WIDTH, HEIGHT);
	int window = glutCreateWindow("Ray Tracer");
#ifdef __APPLE__
	// This is needed on recent Mac OS X versions to correctly display the window.
	glutReshapeWindow(WIDTH - 1, HEIGHT - 1);
#endif
	glutDisplayFunc(display);
	glutIdleFunc(idle);
	init();
	glutMainLoop();
}

