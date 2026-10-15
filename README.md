# raytracer-cpp

Un trazador de rayos escrito desde cero en C++20, construido paso a paso como proyecto del curso **COMP 6838 — Computer Graphics: From Foundations to Generative AI**. Acompaña al capítulo *Raytracing* de las notas del curso (`02-notes/md/raytracing/`): cada pieza de código tiene su contraparte teórica ahí, y este README traza esa correspondencia.

No usa ninguna biblioteca externa de gráficos ni de álgebra lineal — todo, desde los vectores hasta el escritor de PNG, está implementado en el propio proyecto, con la intención explícita de que cada paso sea trazable a las matemáticas que lo sustentan.

## Matemáticas y física en el ray tracing

El ray tracing es una técnica de computación gráfica que genera representaciones 3D en pantallas 2D simulando el recorrido de la luz. En él convergen varias áreas de la matemática y la física, y cada una tiene un lugar concreto en este proyecto:

- **Cálculo vectorial y matemáticas discretas** — permiten representar objetos mediante funciones (superficies paramétricas) y resolver intersecciones rayo-superficie. En este proyecto, todo objeto se discretiza como una malla de triángulos (`engine/parametric_surfaces.hpp`, `engine/polyhedra.hpp`, `engine/flower.hpp`), y la intersección rayo-triángulo se resuelve en forma cerrada, sin iteración, vía el algoritmo de Möller–Trumbore (`physics/triangle.hpp`).
- **Álgebra lineal** — se usa para calcular normales (producto cruz), reflejar direcciones, y para el andamiaje de transformaciones afines (`engine/vec3.hpp`, `engine/mat3.hpp`, `engine/transform.hpp`) que posiciona y orienta cada objeto en la escena.
- **Modelos físicos** — describen la interacción luz-materia (ley del coseno de Lambert, reflexión especular de Phong, reflexión especular perfecta) y permiten asignar color, sombras y reflejos a cada punto visible (`shading/lighting.hpp`, `shading/render.hpp`).
- **Ecuaciones diferenciales y análisis numérico** — aunque este proyecto resuelve la intersección rayo-superficie en forma cerrada para todas sus primitivas (evitando Newton–Raphson, ver más abajo), el marco teórico completo de las notas del curso cubre el caso general, incluido el método de Newton para superficies sin solución cerrada.

**Resultado:** el algoritmo de ray tracing convierte esos principios en representaciones 3D visualizables en pantallas 2D, calculando para cada píxel qué objeto es visible, con qué color, y bajo qué iluminación.

## Modelo matemático condensado

Esta sección resume las fórmulas que el código implementa, en el orden en que un rayo las atraviesa. La notación y la numeración de algoritmos coinciden con las notas del curso (`02-notes/md/raytracing/`). Los vectores unitarios se escriben con sombrero ($\hat D$, $\hat N$, $\hat L$, $\hat V$).

### El rayo

Un rayo es la semirrecta

$$P(t) = O + t\,\hat D, \qquad |\hat D| = 1, \qquad t \ge 0$$

con $O$ el origen y $\hat D$ la dirección, siempre normalizada por convención de todo el proyecto: así $t$ es directamente una distancia, no un parámetro adimensional. `engine::Ray` es exactamente este par $(O, \hat D)$. Toda búsqueda de intersección se hace sobre un intervalo semiabierto $[t_{min}, t_{max})$, no solo $t \ge 0$, para poder acotar la búsqueda (un rayo de sombra, por ejemplo, solo debe buscar hasta la distancia de la luz).

### Intersección rayo-triángulo (Möller–Trumbore)

Para un triángulo $(v_0, v_1, v_2)$, un punto interior se escribe en coordenadas baricéntricas

$$Q(u,v) = v_0 + u\,e_1 + v\,e_2, \qquad e_1 = v_1 - v_0, \quad e_2 = v_2 - v_0$$

Igualando $O + t\hat D = Q(u,v)$ se obtiene un sistema lineal $3 \times 3$ que se resuelve por la regla de Cramer usando $\det[a\ b\ c] = a \cdot (b \times c)$:

$$t = \frac{e_2 \cdot q}{\Delta}, \qquad u = \frac{T \cdot p}{\Delta}, \qquad v = \frac{\hat D \cdot q}{\Delta}$$

con

$$p = \hat D \times e_2, \qquad T = O - v_0, \qquad q = T \times e_1, \qquad \Delta = e_1 \cdot p$$

El impacto es válido si $u \ge 0$, $v \ge 0$, $u + v \le 1$ y $t \in [t_{min}, t_{max})$. Es una fórmula cerrada, sin iteración: el método de Newton–Raphson de las notas (§2.1.4) no se necesita porque toda superficie de este proyecto ya llega discretizada en triángulos.

Los límites baricéntricos se ensanchan por una tolerancia `bary_eps` (por defecto $10^{-9}$): en un vértice compartido por varios triángulos, $u = 0$ o $v = 0$ exactamente, y el redondeo de punto flotante puede empujar a *todos* los triángulos vecinos fuera de su propio rango simultáneamente, produciendo una fuga (un rayo que debería impactar no impacta nada). Una auditoría con miles de rayos apuntando exactamente a cada vértice y arista compartida de cada primitiva midió fugas de hasta 37 % sin esta tolerancia, y cero con ella.

### Mallas y el filtro de degenerados

Toda primitiva (`sphere`, `cylinder`, `cone`, `torus`, `cube`, `tetrahedron`, `star`, `flower`, ...) se genera como una malla de triángulos y se envuelve en un `physics::Mesh`, que filtra los triángulos de área nula que aparecen cuando una fila de la parametrización colapsa a un punto (el polo de una esfera, el ápice de un cono, el centro de la flor en $r = 0$). Con el área

$$A = \tfrac{1}{2}\,\left| (v_1 - v_0) \times (v_2 - v_0) \right|$$

se descarta todo triángulo con $A < \varepsilon$. `Mesh::intersect` recorre por fuerza bruta todos los triángulos y se queda con el $t$ mínimo; `Mesh::occluded` sale en el primer impacto, sin buscar el más cercano, porque solo importa si algo bloquea, no qué ni a qué distancia.

### Iluminación local (Algoritmo 2.7: ambiental + Lambert + Phong + sombras)

En el punto de impacto $P$ con normal unitaria $\hat N$, para cada luz $j$ con dirección unitaria $\hat L_j$ e intensidad $I_j$:

$$I = I_A + \sum_{j:\ \hat N \cdot \hat L_j > 0} V_j \left[ I_j\,(\hat N \cdot \hat L_j) + I_j\,\max(0,\ \hat R_j \cdot \hat V)^{s} \right]$$

donde

$$\hat R_j = 2\,(\hat N \cdot \hat L_j)\,\hat N - \hat L_j$$

es la dirección espejo de la luz (`engine::reflect`), $\hat V = -\hat D$ es la dirección hacia el observador, $s$ es el exponente especular de Phong del material (si el material no tiene exponente, el término especular se omite) y $V_j \in \{0, 1\}$ es el **factor de visibilidad**: vale 0 si un rayo de sombra desde $P$ hacia la luz encuentra un obstáculo antes de llegar. Ese rayo usa $t_{min} = \delta$ (para evitar el *shadow acne*, el autoimpacto espurio del propio triángulo por redondeo) y $t_{max} = |L_j|$ para una luz puntual, o $+\infty$ para una direccional.

El color local es $c_{loc} = \rho\, I$, con $\rho$ el albedo del material. El modelo del curso fija $\rho_s = \rho_d = \rho$, así que un solo color tiñe tanto el término difuso como el especular.

### Reflejos recursivos (Algoritmo 2.9, trazado de rayos de Whitted)

Si el material tiene reflectividad $r > 0$ y queda presupuesto de recursión $d > 0$:

$$c = \mathrm{sat}\left( (1 - r)\,c_{loc} + r\,c_{refl} \right), \qquad c_{refl} = \mathrm{trace}(P,\ R,\ \delta,\ \infty,\ d - 1)$$

con la dirección reflejada

$$R = \hat D - 2\,(\hat N \cdot \hat D)\,\hat N$$

y $\mathrm{sat}(x) = \min(\max(x, 0), 1)$ aplicada canal a canal. La cota de profundidad $d_{max}$ no es una optimización: es lo único que impide una recursión infinita ante dos espejos enfrentados con $r = 1$ (el "pasillo de espejos"). El error de truncar en profundidad $d$ decae geométricamente como $r^{d+1}$ (medido directamente en este proyecto: la razón entre errores de profundidades consecutivas da $r$ con precisión de máquina).

### Cámara

Con origen $O$, punto de interés $T$ y referencia "arriba" $\mathrm{up}$, se construye una base ortonormal a derechas:

$$\hat w = \frac{O - T}{|O - T|}, \qquad \hat u = \frac{\mathrm{up} \times \hat w}{|\mathrm{up} \times \hat w|}, \qquad \hat v = \hat w \times \hat u$$

El rayo del píxel $(i, j)$ de una imagen $N_x \times N_y$, con campo de visión vertical $\theta$, es la normalización de

$$-\hat w + \left(a - \tfrac{1}{2}\right) W\,\hat u + \left(b - \tfrac{1}{2}\right) H\,\hat v$$

con

$$a = \frac{i + 1/2}{N_x}, \qquad b = \frac{j + 1/2}{N_y}, \qquad H = 2\tan\frac{\theta}{2}, \qquad W = H\,\frac{N_x}{N_y}$$

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

## Dependencias

El proyecto no usa bibliotecas externas: solo necesita una cadena de herramientas de C++ moderna.

| Requisito | Versión mínima | Para qué |
|---|---|---|
| Compilador con C++20 | GCC 10, Clang 10 (con libstdc++ 10 o libc++ 12+), o MSVC 2019 16.8 | `<numbers>`, inicializadores designados (`Material{.albedo = ...}`), `<filesystem>` |
| CMake | 3.20 | sistema de construcción y `ctest` |
| Make o Ninja | cualquiera | generador de CMake |
| OpenMP | opcional | render multinúcleo; sin él todo funciona en un solo hilo |

No hace falta Python, ni GPU, ni ninguna biblioteca instalada aparte. El código solo usa la biblioteca estándar.

Comprobar lo que hay instalado:

```bash
g++ --version
cmake --version
```

Instalación según el sistema:

```bash
sudo apt install build-essential cmake ninja-build
```

Esa línea sirve para Ubuntu, Debian y WSL (incluye `libgomp`, el runtime de OpenMP de GCC). En Arch Linux:

```bash
sudo pacman -S base-devel cmake ninja
```

En macOS, el Clang de Apple no trae OpenMP; para el render multinúcleo hay que instalarlo con Homebrew:

```bash
brew install cmake libomp
```

Si `libomp` no es detectado, CMake lo avisa y el programa corre en un solo hilo.

**Windows.** La opción más segura es WSL (Ubuntu) con los comandos de arriba. Con MinGW-w64 (MSYS2) también funciona, usando GCC y CMake de MSYS2. Con Visual Studio (MSVC) se necesita la edición 2019 16.8 o posterior, y hay una limitación conocida: MSVC solo implementa OpenMP 2.0 y el bucle de píxeles usa la cláusula `collapse(2)` de OpenMP 3.0, por lo que ese camino puede no compilar sin ajustar la directiva en `shading/render.hpp`.

**Equipos de laboratorio o universidad.** Si no hay permisos de administrador, se puede construir con un compilador instalado en el directorio del usuario (por ejemplo con `conda install -c conda-forge cxx-compiler cmake ninja`). Un error de CMake sobre `cmake_minimum_required` indica una versión de CMake menor que 3.20; errores sobre `numbers`, `designated initializers` o `-std=c++20` indican un compilador demasiado antiguo.

CMake imprime una advertencia `OpenMP not found: rendering will run single-threaded` cuando no encuentra OpenMP. No es un error, solo una pérdida de velocidad.

## Compilar y ejecutar

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

`CMakeLists.txt` fija `CMAKE_BUILD_TYPE=Release` por defecto si no se indica otro — importante en este proyecto en particular, porque gran parte de la aritmética caliente vive en funciones `inline` diminutas (`Vec3::operator+`, `dot`, `cross`, `intersect_triangle`) que dependen de optimización para no convertirse en llamadas de función reales dentro de bucles que corren miles de millones de veces por render.

### Paralelismo (OpenMP)

El bucle de píxeles de `shading::render` está paralelizado con OpenMP (`collapse(2)`, `schedule(dynamic)`). El resultado es idéntico bit a bit con cualquier número de hilos, y `tests/test_parallel.cpp` lo verifica. El número de hilos se controla con la variable de entorno estándar:

```bash
OMP_NUM_THREADS=4 ./build/raytracer --mode whitted
```

Medido en una máquina de 28 núcleos (WSL2, Release, modo `whitted`):

| Escena | Resolución | 1 hilo | Todos los hilos | Aceleración |
|---|---|---|---|---|
| galería | 400 × 400 | 11.71 s | 0.91 s | 12.9× |
| flor | 400 × 400 | 38.58 s | 2.47 s | 15.6× |
| flor | 1020 × 1020 | — | 16.84 s | — |

Si se cambia el compilador o la configuración de una carpeta `build/` ya existente, conviene borrarla (`rm -rf build`) y reconfigurar: una caché antigua puede ocultar OpenMP.

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
| `distance` | gris según la distancia `t` (blanco cerca, negro lejos) | no |
| `object-id` | un color fijo por objeto, sobre negro | no |
| `lambert` | albedo × (ambiental + difuso), con sombras | sí |
| `phong` | lambert + brillo especular, con sombras | sí |
| `whitted` | phong + reflejos recursivos (Algoritmo 2.9) | sí |

`normals`, `distance` y `object-id` son mapas de depuración puramente geométricos — útiles para verificar que la malla y la cámara están bien antes de fiarse de la iluminación.

## Pruebas

```bash
ctest --test-dir build --output-on-failure
```

Trece suites, cada una enlazada solo con las capas que le corresponden (así una prueba de una capa inferior no puede depender en secreto de una superior):

- **`engine`, `geometry`** — álgebra, generadores de malla (incluida `flower`), topología, `Color::saturate`, el módulo matemático `floor_mod` (necesario para la flor: `std::fmod` de C++ no coincide con `numpy.mod` para dividendos negativos).
- **`physics`, `watertightness`** — Möller–Trumbore, filtro de degenerados, y la auditoría de fugas en vértices/aristas compartidos de las siete primitivas cerradas.
- **`scene_shading`, `scene_data`** — cámara, escena, materiales y luces con validación en la entrada.
- **`lighting`** — el modelo de iluminación completo: casos exactos de Lambert, el ancho de lóbulo de Phong contra la fórmula $1/\sqrt s$ del texto, sombras (incluida una demostración medida de *shadow acne* con y sin tolerancia), y reflejos (terminación con dos espejos enfrentados, decaimiento geométrico del error de truncamiento).
- **`parallel`** — el render con 1 hilo y con varios produce píxeles idénticos (se omite si no hay OpenMP).
- **`io`, `cli`** — los escritores PPM/PNG (con un decodificador PNG independiente, no el propio escritor, para verificar bit a bit) y el analizador de línea de comandos.
- **`layering`** — que ninguna capa incluya una superior.
- **`outputs`, `flower_outputs`** — pruebas de extremo a extremo que ejecutan los binarios reales y revisan los archivos que escriben.

## Limitaciones conocidas

- **Sin estructura de aceleración.** Toda intersección es fuerza bruta sobre todos los triángulos de todos los objetos. Es aceptable para las primitivas simples de la galería, pero `--mode whitted` sobre una malla de decenas de miles de triángulos (como la flor) es costoso, aun con OpenMP. Antes de asumir que el algoritmo es el cuello de botella, verificar `CMAKE_BUILD_TYPE=Release`. La siguiente fase del proyecto (CUDA/OptiX, con BVH) está descrita en `02-notes/md/raytracing/handoff-cuda-optix.md`.
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
tests/          trece suites de ctest, más los scripts .cmake de capas y salidas
media/          renders de referencia de cada etapa del proyecto
(fuera de este repo) 02-notes/md/raytracing/  las notas teóricas que este código implementa
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
