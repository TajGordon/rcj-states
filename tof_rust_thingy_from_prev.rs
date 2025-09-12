use raylib::prelude::*;
use std::{cmp::Ordering, f32::consts::PI};

struct Bot {
    pos: Vector2,
    est_pos: Vector2, /* the estimated position by the algorithm */
    rad: f32,
    color: Color,
    rot: f32, /* points east by default 0 degrees = east; using radians instead of degrees in this program */
    est_rot: f32,
    dir: Vector2, /* dependant on rot, {cos(rot), sin(rot)} */
    est_dir: Vector2,
    num_rays: u8,
    ray_angle: f32, /* dependant on num_rays, just 2PI/num_rays */
    tof_data: Vec<f32>,
}
impl Bot {
    fn rotate(&mut self, amount: f32) {
        self.rot += amount;
        /* update direction vector */
        self.dir.x = self.rot.cos();
        self.dir.y = self.rot.sin();
    }
    fn cast_rays(&mut self, step_size: f32, max_steps: i32, walls: &[Wall; 10]) {
        /* simple little iterative approach for now */
        for ray in 0..self.num_rays as usize {
            let mut ray_pos = self.pos;
            let ray_angle = self.rot + ray as f32 * self.ray_angle;
            let ray_dir = Vector2::new(ray_angle.cos(), ray_angle.sin());
            'raycasting: for i in 0..max_steps {
                ray_pos += ray_dir * step_size;
                /* ray collision check */
                for wall in walls {
                    if ray_pos.x < wall.b.x
                        && ray_pos.y < wall.b.y
                        && ray_pos.x > wall.a.x
                        && ray_pos.y > wall.a.y
                    {
                        break 'raycasting;
                    }
                }
            }
            self.tof_data[ray] = (ray_pos - self.pos).length();
        }
    }
    fn est_pos(&mut self) {
        
    }
}
struct Wall {
    a: Vector2,
    b: Vector2,
    color: Color,
}
struct Field {
    walls: [Wall; 10],
}
enum RenderMode {
    Real,
    Simulated,
}

#[rustfmt::skip]
fn main() {
    let mut screen_width = 1500;
    let mut screen_height = 900;
    let (mut rl, thread) = raylib::init()
        .size(screen_width, screen_height)
        .title("TOF localization")
        .resizable()
        .build();

    rl.set_target_fps(144);

    /* constants */
    let shift_speed_multiplier = 5.; /* flat multiplier of 5x for holding shift */

    let bot_move_speed = 450.;
    let bot_rotate_speed = PI / 2.;

    let camera_zoom_speed: f32 = 4.;

    let mut bot = Bot {
        pos: Vector2::new(2430./2., 1820./2.),
        est_pos: Vector2::zero(),
        rad: 110.,
        color: Color::CORNFLOWERBLUE,
        rot: 0.,
        est_rot: 0.,
        dir: Vector2::new(1., 0.),
        est_dir: Vector2::new(1., 0.),
        num_rays: 8,
        ray_angle: 2. * PI / 8.,
        tof_data: vec![-1.; 8],
    };

    let field = Field {
        /* currently not coloring in the goals, because not doing any vision using this - maybe will do that in like unity or something */
        walls: [
            /* walls */
            Wall { a: Vector2::new(-10., -10.),             b: Vector2::new(2440., 0.),             color: Color::BLACK},
            Wall { a: Vector2::new(-10., 0.),               b: Vector2::new(0., 1820.),             color: Color::BLACK},
            Wall { a: Vector2::new(-10., 1820.),            b: Vector2::new(2440., 1830.),          color: Color::BLACK},
            Wall { a: Vector2::new(2430., 0.),              b: Vector2::new(2440., 1820.),          color: Color::BLACK},
            /* goal left */
            Wall { a: Vector2::new(0., 600.),               b: Vector2::new(140., 610.),            color: Color::BLACK},
            Wall { a: Vector2::new(56., 610.),              b: Vector2::new(66., 1210.),            color: Color::BLACK},
            Wall { a: Vector2::new(0., 1210.),              b: Vector2::new(140., 1220.),           color: Color::BLACK},
            /* goal right */
            Wall { a: Vector2::new(2430. - 140., 600.),     b: Vector2::new(2430., 610.),           color: Color::BLACK},
            Wall { a: Vector2::new(2430. - 66., 610.),      b: Vector2::new(2430. - 56., 1210.),    color: Color::BLACK},
            Wall { a: Vector2::new(2430. - 140., 1210.),    b: Vector2::new(2430., 1220.),          color: Color::BLACK},
        ]
    };

    let mut camera = Camera2D {
        target: bot.pos,
        offset: Vector2::new(screen_width as f32 / 2., screen_height as f32 / 2.),
        rotation: 0.,
        zoom: 0.4,
    };

    let mut render_mode = RenderMode::Real;

    while !rl.window_should_close() {
        /* update */
        let dt = rl.get_frame_time();
        let fps = rl.get_fps();
        screen_width = rl.get_screen_width();
        screen_height = rl.get_screen_height();

        /* multiplier if shift is held, else 1 */
        let frame_shift_speed_multiplier = (shift_speed_multiplier * (rl.is_key_down(KeyboardKey::KEY_LEFT_SHIFT) as i32 as f32)) + (1. - (rl.is_key_down(KeyboardKey::KEY_LEFT_SHIFT) as i32 as f32));

        /* switch render mode */
        if rl.is_key_pressed(KeyboardKey::KEY_SPACE) {
            match render_mode {
                RenderMode::Real => render_mode = RenderMode::Simulated,
                RenderMode::Simulated => render_mode = RenderMode::Real,
            }
        }

        /* move bot */
        let dir: Vector2 = Vector2::new((rl.is_key_down(KeyboardKey::KEY_D) as i32 - rl.is_key_down(KeyboardKey::KEY_A) as i32) as f32, (rl.is_key_down(KeyboardKey::KEY_S) as i32 - rl.is_key_down(KeyboardKey::KEY_W) as i32) as f32).normalized();
        if dir != Vector2::zero() {
            let old_pos = bot.pos;
            // bot.pos = bot.pos + (dir * bot_move_speed * frame_shift_speed_multiplier * dt);
            /* split into components for simplicity */
            /* check if new_pos is colliding */
            /* x movement */
            bot.pos.x += dir.x * bot_move_speed * frame_shift_speed_multiplier * dt;
            for wall in &field.walls {
                let r = Rectangle {
                    x: wall.a.x,
                    y: wall.a.y,
                    width: wall.b.x - wall.a.x,
                    height: wall.b.y - wall.a.y,
                };
                if r.check_collision_circle_rec(bot.pos, bot.rad) {
                    bot.pos = old_pos;
                    break;
                }
            }
            /* y movement */
            bot.pos.y += dir.y * bot_move_speed * frame_shift_speed_multiplier * dt;
            for wall in &field.walls {
                let r = Rectangle {
                    x: wall.a.x,
                    y: wall.a.y,
                    width: wall.b.x - wall.a.x,
                    height: wall.b.y - wall.a.y,
                };
                if r.check_collision_circle_rec(bot.pos, bot.rad) {
                    bot.pos = old_pos;
                    break;
                }
            }
        }
        bot.rotate(bot_rotate_speed * (rl.is_key_down(KeyboardKey::KEY_E) as i32 - rl.is_key_down(KeyboardKey::KEY_Q) as i32) as f32 * dt);

        /* move camera */
        if rl.is_mouse_button_down(MouseButton::MOUSE_BUTTON_MIDDLE) {
            /* click-and-drag instead of arrow keys (like fusion) */
            camera.target += -rl.get_mouse_delta()/camera.zoom;
        }
        /* zoom camera */
        if rl.get_mouse_wheel_move() != 0. {
            /* zooms like fusion :) */
            let mpos = rl.get_mouse_position();
            let abc = rl.get_world_to_screen2D(camera.target, camera) - mpos;
            camera.target = rl.get_screen_to_world2D(mpos, camera);
            camera.offset = camera.offset - abc;

            camera.zoom *= (0.5f32).powf(camera_zoom_speed * -rl.get_mouse_wheel_move() * frame_shift_speed_multiplier * dt); /* multiplicative zoom */
            // println!("{}", camera.zoom);
        }

        /* logic */
        bot.cast_rays(1., screen_width + screen_height, &field.walls); /* max distance should theoretically be |(screen_width, screen_height)| */

        /* render */
        let mut d = rl.begin_drawing(&thread);
        d.clear_background(Color::DARKSLATEGRAY);
        /* camera space */
        d.draw_mode2D(camera, |mut d2, _| {
            /* draw field */
            for wall in &field.walls {
                d2.draw_rectangle_v(wall.a, wall.b - wall.a, wall.color);
            }
            /* draw bot */
            match render_mode {
                RenderMode::Real => {
                    d2.draw_circle_v(bot.pos, bot.rad, bot.color);
                    d2.draw_line_ex(bot.pos, bot.pos + bot.dir * 200., 10., Color::GOLDENROD);
                    /* draw rays */
                    for ray in 0..bot.num_rays {
                        d2.draw_line_v(bot.pos, bot.pos + Vector2::new(1., 0.).rotated(bot.rot + bot.ray_angle * ray as f32) * bot.tof_data[ray as usize], Color::AZURE);
                    }
                },
                RenderMode::Simulated => {
                    d2.draw_circle_v(bot.est_pos, bot.rad, bot.color);
                    d2.draw_line_ex(bot.est_pos, bot.est_pos + bot.est_dir * 200., 10., Color::GOLDENROD);
                    /* draw rays */
                    for ray in 0..bot.num_rays {
                        d2.draw_line_v(bot.est_pos, bot.est_pos + Vector2::new(1., 0.).rotated(bot.est_rot + bot.ray_angle * ray as f32) * bot.tof_data[ray as usize], Color::AZURE);
                    }
                },
            }
        });
        /* screen space */
        d.draw_text(&format!("width: {screen_width}, height: {screen_height}, camera.zoom: {}, fps: {fps}, bot.pos: ({:.2},{:.2})", camera.zoom, bot.pos.x, bot.pos.y), 12, 12, 20, Color::ORANGE);
    }
}