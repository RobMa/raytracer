#pragma once
#include "global.hpp"

enum class MaterialColor {
	Red, Green, Blue, White
};

typedef struct {
	enum  {
		Shiny = 1,
		Reflective = 2
	};
}MaterialOption;

class Material
{
public:
	Material(const RGBd& ambient_reflection, const RGBd& diffuse_reflection, const RGBd& specular_reflection, double shininess, const RGBd& coherent_reflection);

	RGBd& ambient_reflection();
	const RGBd& ambient_reflection() const;

	RGBd& diffuse_reflection();
	const RGBd& diffuse_reflection() const;

	RGBd& specular_reflection();
	const RGBd& specular_reflection() const;


	double& shininess();
	double shininess() const;

	RGBd& coherent_reflection();
	const RGBd& coherent_reflection() const;
	
	static Material Generator(MaterialColor c, int opt);

private:
	RGBd ambient_reflection_;
	RGBd diffuse_reflection_;
	RGBd specular_reflection_;
	RGBd coherent_reflection_;
	double shininess_;
};