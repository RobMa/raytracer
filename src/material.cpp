#include "material.hpp"

//const Material Material::DEFAULT = Material{
//	RGBd{ 1,0,0 }, // ambient-reflection-color
//	RGBd{ 1,0,0 }, // diffuse-reflection-color
//	RGBd{ 1,1,1 }, // specular-reflection-color
//	10, // shininess
//	RGBd{ 0,0,0 } // coherent-reflection color
//};
//const Material Material::REFLECTIVE = Material{
//	RGBd{ 0,1,0 }, // ambient-reflection-color
//	RGBd{ 0,1,0 }, // diffuse-reflection-color
//	RGBd{ 1,1,1 }, // specular-reflection-color
//	15, // shininess
//	RGBd{ 1.5, 0.4, 0.4 } // coherent-reflection color
//};


Material::Material(const RGBd & ambient_reflection, const RGBd & diffuse_reflection, const RGBd & specular_reflection, double shininess, const RGBd & coherent_reflection) :
	ambient_reflection_{ ambient_reflection },
	diffuse_reflection_{ diffuse_reflection },
	specular_reflection_{ specular_reflection },
	shininess_{ shininess },
	coherent_reflection_{ coherent_reflection }
{
}

RGBd & Material::ambient_reflection()
{
	return ambient_reflection_;
}

const RGBd & Material::ambient_reflection() const
{
	return ambient_reflection_;
}

RGBd & Material::diffuse_reflection()
{
	return diffuse_reflection_;
}

const RGBd & Material::diffuse_reflection() const
{
	return diffuse_reflection_;
}

RGBd & Material::specular_reflection()
{
	return specular_reflection_;
}

const RGBd & Material::specular_reflection() const
{
	return specular_reflection_;
}

double & Material::shininess()
{
	return shininess_;
}

double Material::shininess() const
{
	return shininess_;
}

RGBd & Material::coherent_reflection()
{
	return coherent_reflection_;
}

const RGBd & Material::coherent_reflection() const
{
	return coherent_reflection_;
}


Material Material::Generator(MaterialColor c, int opt)
{
	RGBd rgb{ 1,1,1 };
	switch (c)
	{
	case MaterialColor::Red:
		rgb = RGBd{ 1,0,0 };
		break;
	case MaterialColor::Green:
		rgb = RGBd{ 0,1,0 };
		break;
	case MaterialColor::Blue:
		rgb = RGBd{ 0,0,1 };
		break;
	case MaterialColor::White:
		rgb = RGBd{ 1,1,1 };
		break;
	}

	RGBd spec{ 0,0,0 };
	if (opt & MaterialOption::Shiny) {
		spec << 1, 1, 1;
	}

	RGBd refl{ 0,0,0 };
	if (opt & MaterialOption::Reflective) {
		refl << 1, 1, 1;
		//refl = rgb;
	}

	Material m {
		rgb*0.5, // ambient-reflection-color
		rgb , // diffuse-reflection-color
		spec, // specular-reflection-color
		15, // shininess
		refl*0.6 // coherent-reflection color
	};
	return m;
}

