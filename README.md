# raytracer-cpp

Un trazador de rayos escrito desde cero en C++20, construido paso a paso como proyecto del curso **COMP 6838 — Computer Graphics: From Foundations to Generative AI**. Acompaña al capítulo *Raytracing* de las notas del curso (`01-notes/raytracing/`) y a su desarrollo en el libro del curso (`04-book/`): cada pieza de código tiene su contraparte teórica ahí, y este README traza esa correspondencia.

No usa ninguna biblioteca externa de gráficos ni de álgebra lineal — todo, desde los vectores hasta el escritor de PNG, está implementado en el propio proyecto, con la intención explícita de que cada paso sea trazable a las matemáticas que lo sustentan.

## Matemáticas y física en el ray tracing

El ray tracing es una técnica de computación gráfica que genera representaciones 3D en pantallas 2D simulando el recorrido de la luz. En él convergen varias áreas de la matemática y la física, y cada una tiene un lugar concreto en este proyecto:

- **Cálculo vectorial y matemáticas discretas** — permiten representar objetos mediante funciones (superficies paramétricas) y resolver intersecciones rayo-superficie. En este proyecto, todo objeto se discretiza como una malla de triángulos (`engine/parametric_surfaces.hpp`, `engine/polyhedra.hpp`, `engine/flower.hpp`), y la intersección rayo-triángulo se resuelve en forma cerrada, sin iteración, vía el algoritmo de Möller–Trumbore (`physics/triangle.hpp`).
- **Álgebra lineal** — se usa para calcular normales (producto cruz), reflejar direcciones, y para el andamiaje de transformaciones afines (`engine/vec3.hpp`, `engine/mat3.hpp`, `engine/transform.hpp`) que posiciona y orienta cada objeto en la escena.
- **Modelos físicos** — describen la interacción luz-materia (ley del coseno de Lambert, reflexión especular de Phong, reflexión especular perfecta) y permiten asignar color, sombras y reflejos a cada punto visible (`shading/lighting.hpp`, `shading/render.hpp`).
- **Ecuaciones diferenciales y análisis numérico** — aunque este proyecto resuelve la intersección rayo-superficie en forma cerrada para todas sus primitivas (evitando Newton–Raphson, ver más abajo), el marco teórico completo de las notas del curso cubre el caso general, incluido el método de Newton para superficies sin solución cerrada.

**Resultado:** el algoritmo de ray tracing convierte esos principios en representaciones 3D visualizables en pantallas 2D, calculando para cada píxel qué objeto es visible, con qué color, y bajo qué iluminación.

## Modelo matemático condensado

Esta sección resume las fórmulas que el código implementa, en el orden en que un rayo las atraviesa. La notación y la numeración de algoritmos coinciden con las notas del curso (`01-notes/raytracing/`).

### El rayo

Un rayo es la semirrecta

$$P(t) = O + t\,\hat D,\qquad \|\hat D\| = 1,\qquad t \ge 0,$$

con $O$ el origen y $\hat D$ la dirección, siempre normalizada por convención de todo el proyecto — así $t$ es directamente una distancia, no un parámetro adimensional. `engine::Ray` es exactamente este par $(O,\hat D)$; toda búsqueda de intersección se hace sobre un intervalo semiabierto $[t_{\min}, t_{\max})$, no solo $t\ge0$, para poder acotar la búsqueda (un rayo de sombra, por ejemplo, solo debe buscar hasta la distancia de la luz).

### Intersección rayo-triángulo (Möller–Trumbore)

Para un triángulo $(v_0,v_1,v_2)$, un punto interior se escribe en coordenadas baricéntricas $Q(u,v) = v_0 + u\,e_1 + v\,e_2$ con $e_1=v_1-v_0$, $e_2=v_2-v_0$. Igualando $O+t\hat D=Q(u,v)$ se obtiene un sistema lineal $3\times3$ que se resuelve por la regla de Cramer usando $\det[a\,b\,c]=a\cdot(b\times c)$:

$$t=\frac{e_2\cdot Q_{\text{vec}}}{\det},\qquad u=\frac{T\cdot P_{\text{vec}}}{\det},\qquad v=\frac{\hat D\cdot Q_{\text{vec}}}{\det},$$

con $P_{\text{vec}}=\hat D\times e_2$, $T=O-v_0$, $Q_{\text{vec}}=T\times e_1$, $\det=e_1\cdot P_{\text{vec}}$. El impacto es válido si $u,v\ge0$, $u+v\le1$ y $t\in[t_{\min},t_{\max})$. Es una fórmula cerrada, sin iteración — el método de Newton–Raphson de las notas (§2.1.4) no se necesita porque toda superficie de este proyecto ya llega discretizada en triángulos.

Los límites baricéntricos se ensanchan por una tolerancia `bary_eps` (por defecto $10^{-9}$): en un vértice compartido por varios triángulos, $u=0$ o $v=0$ exactamente, y el redondeo de punto flotante puede empujar a *todos* los triángulos vecinos fuera de su propio rango simultáneamente, produciendo una fuga (un rayo que debería impactar no impacta nada). Una auditoría con miles de rayos apuntando exactamente a cada vértice y arista compartida de cada primitiva midió fugas de hasta 37% sin esta tolerancia, y cero con ella.

### Mallas y el filtro de degenerados

Toda primitiva (`sphere`, `cylinder`, `cone`, `torus`, `cube`, `tetrahedron`, `star`, `flower`, ...) se genera como una malla de triángulos y se envuelve en un `physics::Mesh`, que filtra los triángulos de área nula que aparecen cuando una fila de la parametrización colapsa a un punto (el polo de una esfera, el ápice de un cono, el centro de la flor en $r=0$): con $A=\frac12\|(\,v_1-v_0)\times(v_2-v_0)\|$, se descarta todo triángulo con $A<\varepsilon$. `Mesh::intersect` recorre por fuerza bruta todos los triángulos y se queda con el $t$ mínimo; `Mesh::occluded` sale en el primer impacto, sin buscar el más cercano, porque solo importa si algo bloquea, no qué ni a qué distancia.

### Iluminación local (Algoritmo 2.7: ambiental + Lambert + Phong + sombras)

En el punto de impacto $P$ con normal unitaria $\hat N$, para cada luz $j$ con dirección unitaria $\hat L_j$ e intensidad $I_j$:

$$I = I_A \;+\; \sum_{j\,:\,\hat N\cdot\hat L_j>0} V_j\left[I_j\,(\hat N\cdot\hat L_j) \;+\; I_j\,(\hat R_j\cdot\hat V)^{s}\,[\hat R_j\cdot\hat V>0]\right],$$

donde $\hat R_j = 2(\hat N\cdot\hat L_j)\hat N-\hat L_j$ es la dirección espejo de la luz (`engine::reflect`), $\hat V=-\hat D$ es la dirección hacia el observador, $s$ el exponente especular de Phong del material, y $V_j\in\{0,1\}$ el **factor de visibilidad**: cero si un rayo de sombra desde $P$ hacia la luz, con $t_{\min}=\delta$ (para evitar el *shadow acne*, el autoimpacto espurio del propio triángulo por redondeo) y $t_{\max}=\|L_j\|$ (luz puntual) o $+\infty$ (direccional), encuentra un obstáculo antes de llegar. El color final es $c_{\text{loc}}=\rho\cdot I$, con $\rho$ el albedo del material — el modelo del curso fija $\rho_s=\rho_d=\rho$, así que un solo color tiñe tanto el término difuso como el especular.

### Reflejos recursivos (Algoritmo 2.9, trazado de rayos de Whitted)

Si el material tiene reflectividad $r>0$ y queda presupuesto de recursión $d>0$:

$$c = \operatorname{sat}\!\Big((1-r)\,c_{\text{loc}} + r\cdot\text{trace}(P,\,R,\,\delta,\,\infty,\,d-1)\Big),\qquad R = \hat D - 2(\hat N\cdot\hat D)\hat N,$$

con $\operatorname{sat}(x)=\min(\max(x,0),1)$ aplicado canal a canal. La cota de profundidad $d_{\max}$ no es una optimización — es lo único que impide una recursión infinita ante dos espejos enfrentados con $r=1$ (el "pasillo de espejos"). El error de truncar en profundidad $d$ decae geométricamente como $r^{d+1}$ (medido directamente en este proyecto: la razón entre errores de profundidades consecutivas da $r$ con precisión de máquina).

### Cámara

Con origen $O$, punto de interés $T$, referencia "arriba" $\text{up}$, se construye una base ortonormal a derechas

$$\hat w=\widehat{O-T},\qquad \hat u=\widehat{\text{up}\times\hat w},\qquad \hat v=\hat w\times\hat u,$$

y el rayo del píxel $(i,j)$ de una imagen $N_x\times N_y$ con campo de visión vertical $\theta$ es $\hat D_{ij}=\widehat{-\hat w + (s-\tfrac12)\,\text{ancho}\,\hat u + (t-\tfrac12)\,\text{alto}\,\hat v}$, con $s=(i+\tfrac12)/N_x$, $t=(j+\tfrac12)/N_y$ y $\text{alto}=2\tan(\theta/2)$.

## Arquitectura

El código está organizado en capas con dependencia estrictamente unidireccional, verificada en cada build (`tests/check_layering.cmake`, no solo por convención):

```
engine   →  álgebra pura: Vec3, Mat3, Color, generadores de malla, topología.
             No sabe qué es un rayo, una luz ni un material.
physics  →  intersección: Möller–Trumbore, Mesh (filtro de degenerados,
             fuerza bruta con intervalo). No sabe de cámara ni de iluminación.
scene    →  Camera, Scene (dueña de los Mesh, materiales y luces),
             Material, Light. No sabe cómo se calcula un color.
shading  →  el modelo de iluminación completo y los siete modos de render.
io       →  serializa una Image ya calculada a PPM o PNG. No sabe de
             rayos, escenas ni iluminación.
```

`main.cpp` (la galería) y `flower_main.cpp` (la flor) son los únicos orquestadores: arman una escena, construyen una cámara y llaman a `shading::render`.

## Compilar y ejecutar

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`CMakeLists.txt` fija `CMAKE_BUILD_TYPE=Release` por defecto si no se indica otro — importante en este proyecto en particular, porque gran parte de la aritmética caliente vive en funciones `inline` diminutas (`Vec3::operator+`, `dot`, `cross`, `intersect_triangle`) que dependen de optimización para no convertirse en llamadas de función reales dentro de bucles que corren miles de millones de veces por render.

```bash
./build/raytracer --mode phong --output gallery.png
./build/flower_scene --mode whitted --depth 3 --width 800 --height 800
```

Ambos ejecutables comparten el mismo analizador de opciones:

| Flag | Significado | Por defecto |
|---|---|---|
| `--output PATH` | `.ppm` o `.png` escribe solo ese formato; sin extensión escribe ambos | `gallery` / `flower` |
| `--width`, `--height` | resolución en píxeles | 800 × 300 |
| `--fov` | campo de visión vertical, en grados | 40 |
| `--mode` | ver la tabla de modos abajo | `normals` |
| `--far` | distancia mapeada a negro en modo `distance` | 20 |
| `--depth` | rebotes máximos de reflejo en modo `whitted` | 3 |
| `--help` | muestra esta lista | |

### Modos de render

| Modo | Qué muestra | Usa luces |
|---|---|---|
| `normals` | la normal de la superficie como color, sobre el cielo | no |
| `albedo` | el color plano del material, sin sombrear | no |
| `distance` | gris según la distancia $t$ (blanco cerca, negro lejos) | no |
| `object-id` | un color fijo por objeto, sobre negro | no |
| `lambert` | albedo × (ambiental + difuso), con sombras | sí |
| `phong` | lambert + brillo especular, con sombras | sí |
| `whitted` | phong + reflejos recursivos (Algoritmo 2.9) | sí |

`normals`, `distance` y `object-id` son mapas de depuración puramente geométricos — útiles para verificar que la malla y la cámara están bien antes de fiarse de la iluminación.

## Pruebas

```bash
ctest --test-dir build --output-on-failure
```

Doce suites, cada una enlazada solo con las capas que le corresponden (así una prueba de una capa inferior no puede depender en secreto de una superior):

- **`engine`, `geometry`** — álgebra, generadores de malla (incluida `flower`), topología, `Color::saturate`, el módulo matemático `floor_mod` (necesario para la flor: `std::fmod` de C++ no coincide con `numpy.mod` para dividendos negativos).
- **`physics`, `watertightness`** — Möller–Trumbore, filtro de degenerados, y la auditoría de fugas en vértices/aristas compartidos de las siete primitivas cerradas.
- **`scene_shading`, `scene_data`** — cámara, escena, materiales y luces con validación en la entrada.
- **`lighting`** — el modelo de iluminación completo: casos exactos de Lambert, el ancho de lóbulo de Phong contra la fórmula $1/\sqrt s$ del texto, sombras (incluida una demostración medida de *shadow acne* con y sin tolerancia), y reflejos (terminación con dos espejos enfrentados, decaimiento geométrico del error de truncamiento).
- **`io`, `cli`** — los escritores PPM/PNG (con un decodificador PNG independiente, no el propio escritor, para verificar bit a bit) y el analizador de línea de comandos.
- **`layering`** — que ninguna capa incluya una superior.
- **`outputs`, `flower_outputs`** — pruebas de extremo a extremo que ejecutan los binarios reales y revisan los archivos que escriben.

## Limitaciones conocidas

- **Sin estructura de aceleración.** Toda intersección es fuerza bruta sobre todos los triángulos de todos los objetos. Es aceptable para las primitivas simples de la galería, pero `--mode whitted` sobre una malla de decenas de miles de triángulos (como la flor) es costoso — ver la nota de `CMAKE_BUILD_TYPE` arriba antes de asumir que el algoritmo es el cuello de botella.
- **El suelo de la escena de referencia del capítulo es una esfera de radio 5000; aquí se aproxima con un plano.** Cerca de donde otro objeto toca el suelo, un plano y una esfera gigante no coinciden exactamente, y esa diferencia de forma no se corrige subiendo la resolución de malla (se midió y documentó al validar contra un oráculo analítico independiente).
- **Sin refracción.** El modelo cubre reflexión especular perfecta, no transmisión a través de materiales transparentes — es la extensión natural señalada en el cierre teórico de las notas (§2.1.1, el límite eikonal) pero no implementada.
- **Sin motor de escena persistente.** No hay formato de archivo de escena; cada configuración se escribe directamente en C++ (`main.cpp`, `flower_main.cpp`).

## Estructura de archivos

```
include/raytracer/
  engine/     vec3, mat3, color, image, ray, coordinates, discretizer,
              topology, transform, parametric_surfaces, polyhedra, flower
  physics/    triangle (Möller–Trumbore), mesh
  scene/      camera, scene, material, light
  shading/    shading, lighting, render
  io/         ppm_writer, png_writer, color_bytes
src/
  main.cpp            orquestador de la galería
  flower_main.cpp      orquestador de la flor
  render_options.hpp   analizador de línea de comandos compartido
tests/          doce suites de ctest, más los scripts .cmake de capas y salidas
media/          renders de referencia de cada etapa del proyecto
01-notes/raytracing/   las notas teóricas que este código implementa
```

## Correspondencia con las notas del curso

| Algoritmo / sección | Dónde vive en el código |
|---|---|
| 2.1 — rayo, cámara, viewport | `engine::Ray`, `scene::Camera` |
| 2.1.4 — Newton–Raphson | no usado (todo se discretiza en triángulos; ver arriba) |
| 2.2 — RESPUESTA (intersección) | `physics::intersect_triangle` |
| 2.3 — PRIMER-IMPACTO | `scene::Scene::intersect` |
| 2.4 — ILUMINACIÓN (ambiental + Lambert) | `shading::diffuse_light` |
| 2.3.2 — término especular de Phong | `shading::phong_light` |
| 2.4.2 — OCLUIDO, sombras | `scene::Scene::occluded`, integrado en `accumulate_light` |
| 2.8 — REFLEJAR | `engine::reflect` |
| 2.9 — TRAZAR-RAYO (Whitted) | `shading::trace_ray` |

---

*Curso COMP 6838 — Computer Graphics: From Foundations to Generative AI. Desarrollado paso a paso, con verificación numérica en cada etapa (fórmulas cerradas, oráculos independientes, auditorías de fugas y mutaciones deliberadas sobre los tests) en vez de solo "se ve bien".*
