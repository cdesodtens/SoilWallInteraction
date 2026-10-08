#include <iostream>
#include <fstream>

#include "engine/window_context_handler.hpp"
#include "engine/common/color_utils.hpp"
#include "engine/common/number_generator.hpp"

#include "physics/physics.hpp"
#include "thread_pool/thread_pool.hpp"
#include "renderer/renderer.hpp"

void save_state(const PhysicSolver& solver, const std::string& path) {
    std::ofstream out(path, std::ios::binary);
    if (!out) return;
    uint32_t count = solver.objects.size();
    out.write(reinterpret_cast<const char*>(&count), sizeof(count));
    for (uint32_t i = 0; i < count; i++) {
        out.write(reinterpret_cast<const char*>(&solver.objects.data[i]), sizeof(PhysicObject));
    }
}

bool load_state(PhysicSolver& solver, const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    uint32_t count;
    if (!in.read(reinterpret_cast<char*>(&count), sizeof(count))) return false;
    for (uint32_t i = 0; i < count; i++) {
        PhysicObject obj;
        in.read(reinterpret_cast<char*>(&obj), sizeof(PhysicObject));
        solver.addObject(obj);
    }
    return true;
}

int main()
{
    // ==========================================
    // CONFIGURATION GENERALE
    // ==========================================

    // Taille de la fenêtre graphique affichée à l'écran (pixels)
    const uint32_t window_width = 1920;
    const uint32_t window_height = 1000;

    // Dimensions de l'espace de simulation physique (unités arbitraires)
    const IVec2 simulation_size{960, 540};

    // Configuration des particules
    const uint32_t nbObjects = 5000;
    const uint32_t particles_spawn_per_frame = 50; // Plus ce chiffre est grand, plus le remplissage est rapide
    const float particle_radius_min = 0.4f;
    const float particle_radius_max = 1.4f;
    const float stabilization_time = 10.0f; // Temps de stabilisation après l'émission des particules (secondes)

    // Configuration de l'environnement (mur et toit)
    const float wall_start_position = 700.0f; // Mur à 1/4 de la largeur)
    const float wall_speed = 0.08f;           // Vitesse de déplacement du mur
    const float roof_y = 180.0f;              // Position verticale du toit

    // Configuration de la physique
    const float physics_gravity = 50.0f;      // Force de gravité
    const float physics_damping = 40.0f;      // Amortissement (friction de l'air)
    const float physics_response_coef = 1.0f; // Coefficient de rebond/réponse des collisions
    const float physics_friction_coef = 0.3f; // Frottement inter-particulaire (0.0 lisse, >0 rugueux)
    const uint32_t physics_sub_steps = 8;     // Precision du solveur (plus c'est eleve, plus le sol est rigide/dilatant)
    const float grid_cell_size = 4.0f;        // Taille des cellules de la grille de collision

    // Position du point d'apparition (robinet)
    // On veut tomber au milieu de la zone disponible entre le mur et le bord droit
    const float spawn_x = wall_start_position + (simulation_size.x - wall_start_position) / 2.0f;
    const float spawn_y = roof_y + 80.0f; // Juste sous le toit

    // Fichier de sauvegarde de l'état
    const std::string save_filename = "base_state.bin";
    const bool use_save_file = true;

    // ==========================================
    // INITIALISATION DU MOTEUR
    // ==========================================

    WindowContextHandler app("Verlet-MultiThread", sf::Vector2u(window_width, window_height), sf::Style::Default);
    RenderContext &render_context = app.getRenderContext();

    tp::ThreadPool thread_pool(10);
    PhysicSolver solver{simulation_size, grid_cell_size, thread_pool};
    solver.wallPosition = wall_start_position;
    solver.roof_y = roof_y;
    solver.gravity = {0.0f, physics_gravity};
    solver.velocity_damping = physics_damping;
    solver.response_coef = physics_response_coef;
    solver.friction_coef = physics_friction_coef;
    solver.sub_steps = physics_sub_steps;

    Renderer renderer(solver, thread_pool);

    const float margin = 1.0f;
    const auto zoom = static_cast<float>(window_height - margin) / static_cast<float>(simulation_size.y);
    render_context.setZoom(zoom);
    render_context.setFocus({simulation_size.x * 0.5f, simulation_size.y * 0.5f});

    bool emit = true;
    app.getEventManager().addKeyPressedCallback(sf::Keyboard::Space, [&](sfev::CstEv)
                                                { emit = !emit; });

    constexpr uint32_t fps_cap = 60;
    int32_t target_fps = fps_cap;
    app.getEventManager().addKeyPressedCallback(sf::Keyboard::S, [&](sfev::CstEv)
                                                {
        target_fps = target_fps ? 0 : fps_cap;
        app.setFramerateLimit(target_fps); });

    // Main loop
    const float dt = 1.0f / static_cast<float>(fps_cap);
    float time = 0.0f;

    if (use_save_file && load_state(solver, save_filename)) {
        std::cout << "Etat initial charge depuis " << save_filename << std::endl;
        // On n'a plus besoin d'emettre
        emit = false;
        
        // Mettre à jour la grille car les objets ont été chargés
        solver.addObjectsToGrid();
    } else {
        std::cout << "Demarrage de l'emission et de la stabilisation..." << std::endl;
        
        //EMISSION DES PARTICULES
        while (emit)
        {
            if (solver.objects.size() < nbObjects)
            {
                for (uint32_t i = 0; i < particles_spawn_per_frame; ++i)
                {
                    if (solver.objects.size() >= nbObjects) break;

                    const float random_radius = RNGf::getRange(particle_radius_min, particle_radius_max);
                    
                    float final_spawn_x = spawn_x + RNGf::getRange(-1.5f, 1.5f);
                    float final_spawn_y = spawn_y + RNGf::getRange(-1.5f, 1.5f);

                    const auto id = solver.createObject({final_spawn_x, final_spawn_y}, random_radius);
                    
                    // Vitesse purement verticale vers le bas
                    solver.objects[id].last_position.y -= RNGf::getRange(0.0f, 0.5f);
                    // On met une vitesse horizontale quasi-nulle pour tomber bien droit
                    solver.objects[id].last_position.x -= RNGf::getRange(-0.1f, 0.1f);
                }
            }
            else
            {
                emit = false;
            }
            
            solver.update(dt);
            render_context.clear();
            renderer.render(render_context);
            render_context.display();
        }

        //PHASE DE STABILISATION
        while (time < stabilization_time)
        {
            time += dt;
            solver.update(dt);
            render_context.clear();
            renderer.render(render_context);
            render_context.display();
        }

        // Sauvegarde des positions de référence après stabilisation
        for (auto& obj : solver.objects.data) {
            obj.reference_position = obj.position;
        }

        if (use_save_file) {
            save_state(solver, save_filename);
            std::cout << "Etat initial sauvegarde dans " << save_filename << std::endl;
        }
    }

    // DEPLACEMENT DU MUR
    
    // Vitesse de déplacement du mur (par frame)
    // En positif : vers la droite, en négatif : vers la gauche
    

    while (app.run())
    {
        time += dt;
        solver.wallPosition += wall_speed; 
        solver.update(dt);
        render_context.clear();
        renderer.render(render_context);
        render_context.display();
    }

    return 0;
}
