using Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk.Model;
using System.Linq;
using System.Collections.Generic;
using System;
using System.Security.Cryptography;
using System.IO;
using System.Diagnostics;


namespace Com.CodeGame.CodeBall2018.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        public const double ROBOT_MIN_RADIUS = 1, ROBOT_MAX_RADIUS = 1.05d, ROBOT_MAX_JUMP_SPEED = 15, ROBOT_ACCELERATION = 100, ROBOT_NITRO_ACCELERATION = 30, ROBOT_MAX_GROUND_SPEED = 30, ROBOT_ARENA_E = 0;
        public const double ROBOT_RADIUS = 1, ROBOT_MASS = 2, TICKS_PER_SECOND = 60, MICROTICKS_PER_TICK = 100, RESET_TICKS = 2 * TICKS_PER_SECOND, BALL_ARENA_E = 0.7d, BALL_RADIUS = 2, BALL_MASS = 1, MIN_HIT_E = 0.4d, MAX_HIT_E = 0.5d;
        public const double MAX_ENTITY_SPEED = 100, MAX_NITRO_AMOUNT = 100, START_NITRO_AMOUNT = 50, NITRO_POINT_VELOCITY_CHANGE = 0.6d, NITRO_PACK_X = 20, NITRO_PACK_Y = 1, NITRO_PACK_Z = 30, NITRO_PACK_RADIUS = 0.5d;
        public const double NITRO_PACK_AMOUNT = 100, NITRO_RESPAWN_TICKS = 10 * TICKS_PER_SECOND, GRAVITY = 30;
        public const double ESP = 1E-9, GRAVITYTICK = 0.005, DELTAT = 0;

        static int calcCount = 200;
        static Arena arena;
        static Robot meR;
        static Robot meTest;
        static Rules Myrules;
        static List<BallWay> ballWays;
        static bool isTouchBall;
        static myRobot rTeset = null;
        static myBall bTest = null;
        static List<myAction> actionList;
        static List<myAction> actionList1;
        static List<myAction> actionList2;
        static List<myAction> actionList3;
        static int score1 = 0, score2 = 0;
        private static double xTest = 0;
        private static int lastRobot = 0;
        static int attakerID = 0;
        static myRobot keepEnemy = null;
        private static double goalX = 0;
        static double timeCPU = 0;
        static double AVG = 0;
        static int firstTick = 0;
        static int CalcfirstTick = 0;
        static double firstTickGoalScore = 0;

        static List<calcMove> calcMoves = new List<calcMove>();


        public struct calcMove
        {
            public int tick;
            public double x, y, z;
            public double velX, velY, velZ;
            public double isJump;
            public bool use_nitro;

            public calcMove(int tick, double x, double y, double z, double velX, double velY, double velZ, double isJump, bool use_nitro)
            {
                this.tick = tick;
                this.x = x;
                this.y = y;
                this.z = z;
                this.velX = velX;
                this.velY = velY;
                this.velZ = velZ;
                this.isJump = isJump;
                this.use_nitro = use_nitro;
            }
        }

        public class myAction
        {
            public Model.Action action;
            public int tick;
            public myAction(Model.Action a, int tick)
            {
                this.tick = tick;
                action = new Model.Action
                {
                    target_velocity_x = a.target_velocity_x,
                    target_velocity_y = a.target_velocity_y,
                    target_velocity_z = a.target_velocity_z,
                    jump_speed = a.jump_speed,
                    use_nitro = a.use_nitro
                };
            }
            public myRobot r;
            public myBall ball;
            public double eps = 0.01;
        }

        public static Vec3DPlane min(Vec3DPlane v1, Vec3DPlane v2)
        {
            if (v1.distance > v2.distance)
                return v2;
            return v1;
        }

        public static double dot(Vector3D v1, Vector3D v2)
        {
            return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        }
        public static double dot(Vector3 v1, Vector3 v2)
        {
            return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        }

        public static Vector3D MinusVec(Vector3D v1, Vector3D v2)
        {
            return new Vector3D(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }

        public static Vec3DPlane dan_to_plane(Vector3D point, Vector3D point_on_plane, Vector3D plane_normal)
        {
            Vec3DPlane res = new Vec3DPlane
            {
                distance = dot(MinusVec(point, point_on_plane), plane_normal),
                normal = new Vector3D(plane_normal)
            };
            return res;
        }

        public static Vec3DPlane dan_to_sphere_inner(Vector3D point, Vector3D sphere_center, double sphere_radius)
        {
            Vec3DPlane res = new Vec3DPlane
            {
                distance = sphere_radius - MinusVec(point, sphere_center).Len()
            };
            Vector3D v = MinusVec(sphere_center, point);
            v.normalize();
            res.normal = new Vector3D(v);
            return res;
        }

        public static Vec3DPlane dan_to_sphere_outer(Vector3D point, Vector3D sphere_center, double sphere_radius)
        {
            Vec3DPlane res = new Vec3DPlane
            {
                distance = MinusVec(point, sphere_center).Len() - sphere_radius
            };
            Vector3D v = MinusVec(point, sphere_center);
            v.normalize();
            res.normal = new Vector3D(v);
            return res;
        }

        public static Vec3DPlane dan_to_arena_quarter(Vector3D point)
        {
            Vec3DPlane dan = new Vec3DPlane();
            //Ground
            dan = dan_to_plane(point, new Vector3D(0, 0, 0), new Vector3D(0, 1, 0));

            // Ceiling
            dan = min(dan, dan_to_plane(point, new Vector3D(0.0d, arena.height, 0.0d), new Vector3D(0, -1, 0)));

            // Optimize
            if (point.x < 24 && point.z < 34 && point.y < 13)
                return dan;

            // Side x
            dan = min(dan, dan_to_plane(point, new Vector3D(arena.width / 2, 0, 0), new Vector3D(-1, 0, 0)));

            // Side z (goal)
            dan = min(dan, dan_to_plane(point, new Vector3D(0, 0, (arena.depth / 2) + arena.goal_depth), new Vector3D(0, 0, -1)));

            // Side z
            Vector v = new Vector(point.x, point.y) - new Vector((arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius);
            if (point.x >= (arena.goal_width / 2) + arena.goal_side_radius || point.y >= arena.goal_height + arena.goal_side_radius || (v.x > 0 && v.y > 0 && v.LengthSq >= (arena.goal_top_radius + arena.goal_side_radius) * (arena.goal_top_radius + arena.goal_side_radius)))
                dan = min(dan, dan_to_plane(point, new Vector3D(0, 0, arena.depth / 2), new Vector3D(0, 0, -1)));

            // Side x & ceiling (goal)
            if (point.z >= (arena.depth / 2) + arena.goal_side_radius)
            {
                // x
                dan = min(dan, dan_to_plane(point, new Vector3D(arena.goal_width / 2, 0, 0), new Vector3D(-1, 0, 0)));
                // y
                dan = min(dan, dan_to_plane(point, new Vector3D(0, arena.goal_height, 0), new Vector3D(0, -1, 0)));
            }

            if (point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
            {
                dan = min(dan, dan_to_sphere_inner(
                point, new Vector3D(Clamp(point.x, arena.bottom_radius - (arena.goal_width / 2), (arena.goal_width / 2) - arena.bottom_radius),
                Clamp(point.y, arena.bottom_radius, arena.goal_height - arena.goal_top_radius), (arena.depth / 2) + arena.goal_depth - arena.bottom_radius), arena.bottom_radius));
            }

            // Corner
            if (point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
                dan = min(dan, dan_to_sphere_inner(point, new Vector3D((arena.width / 2) - arena.corner_radius, point.y, (arena.depth / 2) - arena.corner_radius), arena.corner_radius));

            // Goal outer corner
            if (point.z < (arena.depth / 2) + arena.goal_side_radius)
            {
                // Side x
                if (point.x < (arena.goal_width / 2) + arena.goal_side_radius)
                    dan = min(dan, dan_to_sphere_outer(point, new Vector3D((arena.goal_width / 2) + arena.goal_side_radius, point.y, (arena.depth / 2) + arena.goal_side_radius), arena.goal_side_radius));
                // Ceiling
                if (point.y < arena.goal_height + arena.goal_side_radius)
                    dan = min(dan, dan_to_sphere_outer(point, new Vector3D(point.x, arena.goal_height + arena.goal_side_radius, (arena.depth / 2) + arena.goal_side_radius), arena.goal_side_radius));
                // Top corner
                Vector o = new Vector((arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius);
                Vector v1 = new Vector(point.x, point.y) - o;
                if (v1.x > 0 && v1.y > 0)
                {
                    Vector o1 = o + v1.Normalize() * (arena.goal_top_radius + arena.goal_side_radius);
                    dan = min(dan, dan_to_sphere_outer(
                    point, new Vector3D(o1.x, o1.y, (arena.depth / 2) + arena.goal_side_radius),
                    arena.goal_side_radius));
                }
            }
            // Goal inside top corners

            if (point.z > (arena.depth / 2) + arena.goal_side_radius && point.y > arena.goal_height - arena.goal_top_radius)
            {
                // Side x
                if (point.x > (arena.goal_width / 2) - arena.goal_top_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D((arena.goal_width / 2) - arena.goal_top_radius, arena.goal_height - arena.goal_top_radius, point.z), arena.goal_top_radius));
                // Side z
                if (point.z > (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D(point.x, arena.goal_height - arena.goal_top_radius, (arena.depth / 2) + arena.goal_depth - arena.goal_top_radius), arena.goal_top_radius));
            }
            // Bottom corners
            if (point.y < arena.bottom_radius)
            {
                // Side x
                if (point.x > (arena.width / 2) - arena.bottom_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D((arena.width / 2) - arena.bottom_radius, arena.bottom_radius, point.z), arena.bottom_radius));
                // Side z
                if (point.z > (arena.depth / 2) - arena.bottom_radius && point.x >= (arena.goal_width / 2) + arena.goal_side_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D(point.x, arena.bottom_radius, (arena.depth / 2) - arena.bottom_radius), arena.bottom_radius));
                // Side z (goal)
                if (point.z > (arena.depth / 2) + arena.goal_depth - arena.bottom_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D(point.x, arena.bottom_radius, (arena.depth / 2) + arena.goal_depth - arena.bottom_radius), arena.bottom_radius));
                // Goal outer corner
                Vector o = new Vector((arena.goal_width / 2) + arena.goal_side_radius, (arena.depth / 2) + arena.goal_side_radius);
                Vector v1 = new Vector(point.x, point.z) - o;
                if (v1.x < 0 && v1.y < 0 && v1.LengthSq < (arena.goal_side_radius + arena.bottom_radius) * (arena.goal_side_radius + arena.bottom_radius))
                {
                    Vector o1 = o + v1.Normalize() * (arena.goal_side_radius + arena.bottom_radius);
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D(o1.x, arena.bottom_radius, o1.y), arena.bottom_radius));
                }
                // Side x (goal)
                if (point.z >= (arena.depth / 2) + arena.goal_side_radius && point.x > (arena.goal_width / 2) - arena.bottom_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D((arena.goal_width / 2) - arena.bottom_radius, arena.bottom_radius, point.z), arena.bottom_radius));
                // Corner
                if (point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
                {
                    Vector corner_o = new Vector((arena.width / 2) - arena.corner_radius, (arena.depth / 2) - arena.corner_radius);
                    Vector n = new Vector(point.x, point.z) - corner_o;
                    double dist = n.Length;
                    if (dist > arena.corner_radius - arena.bottom_radius)
                    {
                        n = n / dist;
                        Vector o2 = corner_o + n * (arena.corner_radius - arena.bottom_radius);
                        dan = min(dan, dan_to_sphere_inner(point, new Vector3D(o2.x, arena.bottom_radius, o2.y), arena.bottom_radius));
                    }
                }
            }
            // Ceiling corners
            if (point.y > arena.height - arena.top_radius)
            {
                // Side x
                if (point.x > (arena.width / 2) - arena.top_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D((arena.width / 2) - arena.top_radius, arena.height - arena.top_radius, point.z), arena.top_radius));
                // Side z
                if (point.z > (arena.depth / 2) - arena.top_radius)
                    dan = min(dan, dan_to_sphere_inner(point, new Vector3D(point.x, arena.height - arena.top_radius, (arena.depth / 2) - arena.top_radius), arena.top_radius));
                // Corner
                if (point.x > (arena.width / 2) - arena.corner_radius && point.z > (arena.depth / 2) - arena.corner_radius)
                {
                    Vector corner_o = new Vector((arena.width / 2) - arena.corner_radius, (arena.depth / 2) - arena.corner_radius);
                    Vector dv = new Vector(point.x, point.z) - corner_o;
                    if (dv.LengthSq > (arena.corner_radius - arena.top_radius) * (arena.corner_radius - arena.top_radius))
                    {
                        Vector n = dv.Normalize();
                        Vector o2 = corner_o + n * (arena.corner_radius - arena.top_radius);
                        dan = min(dan, dan_to_sphere_inner(point, new Vector3D(o2.x, arena.height - arena.top_radius, o2.y), arena.top_radius));
                    }
                }
            }
            return dan;
        }
        public static Vec3DPlane dan_to_arena(Vector3D point)
        {
            bool negate_x = point.x < 0;
            bool negate_z = point.z < 0;
            if (negate_x)
                point.x = -point.x;
            if (negate_z)
                point.z = -point.z;
            Vec3DPlane result = dan_to_arena_quarter(point);
            if (negate_x)
                result.normal.x = -result.normal.x;
            if (negate_z)
                result.normal.z = -result.normal.z;
            return result;
        }

        static public double Clamp(double v1, double v2)
        {
            if (v1 > v2)
                return v2;
            return v1;
        }
        static public double Clamp(double v1, double v2, double v3)
        {
            return Math.Min(Math.Max(v1, v2), v3);
        }

        static public Vector3 Clamp(Vector3 v, double max)
        {
            if (v.LengthSq > max * max)
            {
                v.Clamp(max);
            }
            return v;
        }

        public class Vec3DPlane
        {
            public double distance;
            public Vector3D normal;
        }

        public class Vector3
        {
            public double x;
            public double y;
            public double z;

            public void Clamp(double max)
            {
                if (LengthSq > max * max)
                {
                    this.Norm();
                    x = x * max;
                    y = y * max;
                    z = z * max;
                }
            }


            public Vector3(double x, double y, double z)
            {
                this.x = x;
                this.y = y;
                this.z = z;
            }
            public Vector3(Vector3 v)
            {
                this.x = v.x;
                this.y = v.y;
                this.z = v.z;
            }

            public Vector3(double value, Vector3 direction)
            {
                double k = value / direction.Length;
                this.x = k * direction.x;
                this.y = k * direction.y;
                this.z = k * direction.z;

            }

            public void Norm()
            {
                double k = Length;
                if (k == 0)
                    return;
                x = x / k;
                y = y / k;
                z = z / k;

            }



            public static Vector3 operator +(Vector3 v1, Vector3 v2)
            {
                return new Vector3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
            }

            public static Vector3 operator -(Vector3 v1, Vector3 v2)
            {
                return new Vector3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
            }

            public static Vector3 operator /(Vector3 v, double k)
            {
                return new Vector3(v.x / k, v.y / k, v.z / k);
            }

            public static Vector3 operator *(double k, Vector3 v)
            {
                return new Vector3(v.x * k, v.y * k, v.z * k);
            }
            public static Vector3 operator *(Vector3 v, double k)
            {
                return new Vector3(v.x * k, v.y * k, v.z * k);
            }

            public static Vector3 operator +(double k, Vector3 v)
            {
                return new Vector3(v.x + k, v.y + k, v.z + k);
            }



            public double Length
            {
                get
                {
                    return (double)Math.Sqrt(x * x + y * y + z * z);
                }
            }
            public double LengthSq
            {
                get
                {
                    return (double)(x * x + y * y + z * z);
                }
            }

            public static Vector3 Zero
            {
                get { return new Vector3(0, 0, 0); }
            }

            public override string ToString()
            {
                return string.Format("X={0}, Y={1} , Z={2}", x, y, z);
            }

        }

        public class Vector3D
        {
            public Vector3D(double X, double Y, double Z)
            {
                x = X; y = Y; z = Z;
            }
            public Vector3D(Vector3D v)
            {
                x = v.x; y = v.y; z = v.z;
            }

            public double x, y, z;
            public double Len()
            {
                return Math.Sqrt(x * x + y * y + z * z);
            }
            public double LenSq()
            {
                return (x * x + y * y + z * z);
            }
            public void normalize()
            {
                double len = this.Len();
                x = x * (1 / len);
                y = y * (1 / len);
                z = z * (1 / len);
            }

        }
        public class Vector
        {
            public double x;
            public double y;

            public Vector(double x, double y)
            {
                this.x = x;
                this.y = y;
            }

            public Vector(double value, Vector direction)
            {
                double k = value / direction.Length;
                this.x = k * direction.x;
                this.y = k * direction.y;

            }

            public void Norm()
            {
                double k = Length;
                if (k == 0)
                    return;
                x = x / k;
                y = y / k;

            }

            public Vector Normalize()
            {
                double k = Length;
                return new Vector(x / k, y = y / k);
            }


            public static Vector operator +(Vector v1, Vector v2)
            {
                return new Vector(v1.x + v2.x, v1.y + v2.y);
            }

            public static Vector operator -(Vector v1, Vector v2)
            {
                return new Vector(v1.x - v2.x, v1.y - v2.y);
            }

            public static Vector operator /(Vector v, double k)
            {
                return new Vector(v.x / k, v.y / k);
            }

            public static Vector operator *(double k, Vector v)
            {
                return new Vector(v.x * k, v.y * k);
            }
            public static Vector operator *(Vector v, double k)
            {
                return new Vector(v.x * k, v.y * k);
            }

            public static Vector operator +(double k, Vector v)
            {
                return new Vector(v.x + k, v.y + k);
            }



            public double Length
            {
                get
                {
                    return (double)Math.Sqrt(x * x + y * y);
                }
            }

            public double LengthSq
            {
                get
                {
                    return (double)(x * x + y * y);
                }
            }


            public static Vector Zero
            {
                get { return new Vector(0, 0); }
            }

            public override string ToString()
            {
                return string.Format("X={0}, Y={1}", x, y);
            }

        }

        public static void collide_entities(Entity a, Entity b, double RND = 0.45)
        {
            Random random = new Random(100);
            Vector3 delta_position = b.position - a.position;
            double distance = delta_position.Length;
            double penetration = a.radius + b.radius - distance;
            if (penetration > 0)
            {
                if (a.type == "Robot" && b.type == "Ball")
                    isTouchBall = true;

                double k_a = (1 / a.mass) / ((1 / a.mass) + (1 / b.mass));
                double k_b = (1 / b.mass) / ((1 / a.mass) + (1 / b.mass));
                Vector3 normal = new Vector3(delta_position);
                normal.Norm();
                a.position -= normal * penetration * k_a;
                b.position += normal * penetration * k_b;
                double delta_velocity = dot(b.velocity - a.velocity, normal) - b.radius_change_speed - a.radius_change_speed;
                if (delta_velocity < 0)
                {
                    Vector3 impulse = (1 + RND) * delta_velocity * normal;
                    a.velocity += impulse * k_a;
                    b.velocity -= impulse * k_b;
                }
            }
        }


        public class myRobot
        {
            public int id, player_id;
            public bool isMy;
            public double velocity_x, velocity_y, velocity_z;
            public double radius;
            public double nitro_amout;
            public double x, y, z;
            public bool touch;
            public double touch_normal_x, touch_normal_y, touch_normal_z;
            public Model.Action action;
            public double radius_change_speed;
            public List<myAction> aList;

            public myRobot(Robot R)
            {
                id = R.id;
                player_id = R.player_id;
                isMy = R.is_teammate;
                velocity_x = R.velocity_x;
                velocity_y = R.velocity_y;
                velocity_z = R.velocity_z;
                radius = R.radius;
                nitro_amout = R.nitro_amount;
                x = R.x;
                y = R.y;
                z = R.z;
                touch = R.touch;
                if (R.touch)
                {
                    touch_normal_x = (double)R.touch_normal_x;
                    touch_normal_y = (double)R.touch_normal_y;
                    touch_normal_z = (double)R.touch_normal_z;
                }
                else
                {
                    touch_normal_x = 0;
                    touch_normal_y = 0;
                    touch_normal_z = 0;
                }
                action = new Model.Action();
                aList = new List<myAction>();
            }
            public myRobot(myRobot R)
            {
                id = R.id;
                player_id = R.player_id;
                isMy = R.isMy;
                velocity_x = R.velocity_x;
                velocity_y = R.velocity_y;
                velocity_z = R.velocity_z;
                radius = R.radius;
                nitro_amout = R.nitro_amout;
                x = R.x;
                y = R.y;
                z = R.z;
                touch = R.touch;
                if (R.touch)
                {
                    touch_normal_x = (double)R.touch_normal_x;
                    touch_normal_y = (double)R.touch_normal_y;
                    touch_normal_z = (double)R.touch_normal_z;
                }
                else
                {
                    touch_normal_x = 0;
                    touch_normal_y = 0;
                    touch_normal_z = 0;
                }
                action = new Model.Action();
                action.jump_speed = R.action.jump_speed;
                action.target_velocity_x = R.action.target_velocity_x;
                action.target_velocity_y = R.action.target_velocity_y;
                action.target_velocity_z = R.action.target_velocity_z;
                action.use_nitro = R.action.use_nitro;
            }

            public Vector3 Pos()
            {
                return new Vector3(x, y, z);
            }

            public Vector3 SpeedVec()
            {
                return new Vector3(velocity_x, 0, velocity_z);
            }


            public void Print()
            {
                Console.WriteLine("x={0} y={1}  z={2}  velocity_x = {3} velocity_y={4} velocity_z={5}", x, y, z, velocity_x, velocity_y, velocity_z);
            }
            public void Clamp()
            {
                Vector3D vel = new Vector3D(velocity_x, velocity_y, velocity_z);
                if (vel.LenSq() > MAX_ENTITY_SPEED * MAX_ENTITY_SPEED)
                {
                    vel.normalize();
                    velocity_x = vel.x * MAX_ENTITY_SPEED;
                    velocity_y = vel.y * MAX_ENTITY_SPEED;
                    velocity_z = vel.z * MAX_ENTITY_SPEED;
                }
            }
            public void Move(double delta_time)
            {
                Clamp();
                x = x + velocity_x * delta_time;
                y = y + velocity_y * delta_time;
                z = z + velocity_z * delta_time;
                y = y - GRAVITYTICK * delta_time / 2;
                velocity_y = velocity_y - GRAVITYTICK;
            }
            public void MoveTick(Vector3 target_velocity_change_tmp, Vector3 target_velocity_change_nitro)
            {
                Clamp();
                x = x + (velocity_x - target_velocity_change_tmp.x * 0.495 - target_velocity_change_nitro.x * 0.495) / 60;
                y = y + (velocity_y + target_velocity_change_nitro.y * 0.495) / 60 - 24.75 * 0.000166666666666;
                y = y - 0.5 * (0.000166666666 / 2) - (target_velocity_change_nitro.y * 2 * 0.495) / 60;
                z = z + (velocity_z - target_velocity_change_tmp.z * 0.495 - target_velocity_change_nitro.z * 0.495) / 60;
                velocity_y = velocity_y - GRAVITYTICK * 100;
            }
            public void colliseArena()
            {
                touch = false;
                Vec3DPlane vd = dan_to_arena(new Vector3D(x, y, z));
                double penetration = radius - vd.distance;
                if (penetration > 0)
                {
                    x = x + penetration * vd.normal.x;
                    y = y + penetration * vd.normal.y;
                    z = z + penetration * vd.normal.z;
                    double velocity = dot(new Vector3D(velocity_x, velocity_y, velocity_z), vd.normal) - radius_change_speed;
                    if (velocity < 0)
                    {
                        Vector3 vv = (1 + Myrules.ROBOT_ARENA_E) * velocity * new Vector3(vd.normal.x, vd.normal.y, vd.normal.z);
                        velocity_x = velocity_x - vv.x;
                        velocity_y = velocity_y - vv.y;
                        velocity_z = velocity_z - vv.z;
                        touch = true;
                        touch_normal_x = vd.normal.x;
                        touch_normal_y = vd.normal.y;
                        touch_normal_z = vd.normal.z;
                    }
                    else
                    {
                        touch = false;
                    }
                }
            }
        }

        public class myBall
        {
            public double x, y, z;
            public double velocity_x, velocity_y, velocity_z;
            public double radius;
            public myBall(Ball b)
            {
                x = b.x; y = b.y; z = b.z;
                velocity_x = b.velocity_x;
                velocity_y = b.velocity_y;
                velocity_z = b.velocity_z;
                radius = b.radius;
            }
            public myBall(myBall b)
            {
                x = b.x; y = b.y; z = b.z;
                velocity_x = b.velocity_x;
                velocity_y = b.velocity_y;
                velocity_z = b.velocity_z;
                radius = b.radius;
            }

            public Vector3 Pos()
            {
                return new Vector3(x, y, z);
            }
            public Vector3 SpeedVec()
            {
                return new Vector3(velocity_x, velocity_x, velocity_z);
            }
            public void Print()
            {
                Console.WriteLine("x={0} y={1}  z={2}  velocity_x = {3} velocity_y={4} velocity_z={5}", x, y, z, velocity_x, velocity_y, velocity_z);
            }

            public void Clamp()
            {
                Vector3D vel = new Vector3D(velocity_x, velocity_y, velocity_z);
                if (vel.LenSq() > MAX_ENTITY_SPEED * MAX_ENTITY_SPEED)
                {
                    vel.normalize();
                    velocity_x = vel.x * MAX_ENTITY_SPEED;
                    velocity_y = vel.y * MAX_ENTITY_SPEED;
                    velocity_z = vel.z * MAX_ENTITY_SPEED;
                }
            }
            public void Move(double delta_time)
            {
                Clamp();
                x = x + velocity_x * delta_time;
                y = y + velocity_y * delta_time;
                z = z + velocity_z * delta_time;
                y = y - GRAVITYTICK * delta_time / 2;
                velocity_y = velocity_y - GRAVITYTICK;
            }
            public void MoveTick()
            {
                Clamp();
                x = x + velocity_x / 60;
                y = y + velocity_y / 60 - 24.75 * 0.0001666666666;
                y = y - 0.5 * (0.000166666 / 2);
                z = z + velocity_z / 60;
                //                y = y - GRAVITYTICK *100 * (1/60) / 2;
                velocity_y = velocity_y - GRAVITYTICK * 100;
            }
            public bool colliseArena()
            {
                bool res = false;
                Vec3DPlane vd = dan_to_arena(new Vector3D(x, y, z));
                double penetration = radius - vd.distance;
                if (penetration > 0)
                {
                    res = true;
                    x = x + penetration * vd.normal.x;
                    y = y + penetration * vd.normal.y;
                    z = z + penetration * vd.normal.z;
                    double velocity = dot(new Vector3D(velocity_x, velocity_y, velocity_z), vd.normal);
                    if (velocity < 0)
                    {
                        Vector3 vv = (1 + BALL_ARENA_E) * velocity * new Vector3(vd.normal.x, vd.normal.y, vd.normal.z);
                        velocity_x = velocity_x - vv.x;
                        velocity_y = velocity_y - vv.y;
                        velocity_z = velocity_z - vv.z;
                    }
                }
                return res;
            }
        }

        public class myWord
        {
            public myBall ball;
            public myRobot[] robots;
            public bool isMyGoal;
            public int tick;
            public int myCurId;
            public myWord(Game game)
            {
                ball = new myBall(game.ball);
                robots = new myRobot[game.robots.Length];
                for (int i = 0; i < game.robots.Length; i++)
                {
                    robots[i] = new myRobot(game.robots[i]);
                    if (meR.id == game.robots[i].id)
                        myCurId = i;
                }
                tick = game.current_tick;
            }
            public myRobot getCurrentRobot()
            {
                return robots[myCurId];
            }
            public void Update()
            {
                double delta_time = 1 / TICKS_PER_SECOND;
                double delta_time_tick = delta_time / (double)MICROTICKS_PER_TICK;
                for (int i = 0; i <= (int)MICROTICKS_PER_TICK - 1; i++)
                {
                    foreach (myRobot robot in robots)
                    {
                        Vector3 robot_action_target_velocity = new Vector3(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z);
                        Vector3 robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                        if (robot.touch)
                        {
                            Vector3 target_velocity = new Vector3(robot_action_target_velocity);
                            target_velocity.Clamp(ROBOT_MAX_GROUND_SPEED);
                            target_velocity -= new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) * dot(new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z), target_velocity);
                            Vector3 target_velocity_change = target_velocity - new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                            if (target_velocity_change.LengthSq > 0)
                            {
                                double acceleration = ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal_y);
                                Vector3 target_velocity_change_tmp = new Vector3(target_velocity_change);
                                target_velocity_change_tmp.Norm();
                                target_velocity_change_tmp = target_velocity_change_tmp * acceleration * delta_time_tick;
                                target_velocity_change_tmp.Clamp(target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + target_velocity_change_tmp.x;
                                robot.velocity_y = robot.velocity_y + target_velocity_change_tmp.y;
                                robot.velocity_z = robot.velocity_z + target_velocity_change_tmp.z;
                            }
                        }
                        if (robot.action.use_nitro)
                        {
                            Vector3 target_velocity_change = new Vector3(robot_action_target_velocity - robot_velocity);
                            target_velocity_change.Clamp(robot.nitro_amout * NITRO_POINT_VELOCITY_CHANGE);
                            if (target_velocity_change.LengthSq > 0)
                            {
                                Vector3 target_velocity_change_norm = new Vector3(target_velocity_change);
                                target_velocity_change_norm.Norm();
                                Vector3 acceleration = target_velocity_change_norm * ROBOT_NITRO_ACCELERATION;
                                Vector3 velocity_change = Clamp(acceleration * delta_time_tick, target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + target_velocity_change.x;
                                robot.velocity_y = robot.velocity_y + target_velocity_change.y;
                                robot.velocity_z = robot.velocity_z + target_velocity_change.z;
                                robot.nitro_amout -= velocity_change.Length / NITRO_POINT_VELOCITY_CHANGE;
                            }
                        }
                        robot.Move(delta_time_tick);
                        robot.radius = ROBOT_MIN_RADIUS + (Myrules.ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
                        robot.radius_change_speed = robot.action.jump_speed;
                    }

                    ball.Move(delta_time_tick);
                    ball.colliseArena();
                    for (int k = 0; k < robots.Length - 1; k++)
                        for (int j = 0; j < k; j++)
                        {
                            Entity a = new Entity(robots[k]);
                            Entity b = new Entity(robots[j]);
                            collide_entities(a, b);
                            robots[k].x = a.position.x; robots[k].y = a.position.y; robots[k].z = a.position.z;
                            robots[k].velocity_x = a.velocity.x; robots[k].velocity_y = a.velocity.y; robots[k].velocity_z = a.velocity.z;
                            robots[j].x = b.position.x; robots[j].y = b.position.y; robots[j].z = b.position.z;
                            robots[j].velocity_x = b.velocity.x; robots[j].velocity_y = b.velocity.y; robots[j].velocity_z = b.velocity.z;
                        }
                    foreach (myRobot robot in robots.Where(r => r.id == meR.id))
                    {
                        Entity a = new Entity(robot);
                        Entity b = new Entity(ball);
                        collide_entities(a, b);
                        robot.x = a.position.x; robot.y = a.position.y; robot.z = a.position.z;
                        robot.velocity_x = a.velocity.x; robot.velocity_y = a.velocity.y; robot.velocity_z = a.velocity.z;
                        ball.x = b.position.x; ball.y = b.position.y; ball.z = b.position.z;
                        ball.velocity_x = b.velocity.x;
                        ball.velocity_y = b.velocity.y;
                        ball.velocity_z = b.velocity.z;
                        robot.colliseArena();
                    }
                    if (ball.z > arena.depth / 2 + ball.radius)
                    {
                        break;
                    }
                    if (-ball.z < -arena.depth / 2 - ball.radius)
                    {
                        break;
                    }

                }
            }

            public void Update_fast()
            {
                int MICROTICKS_PER_TICK_fast = 10;
                double delta_time = 1 / TICKS_PER_SECOND;
                double delta_time_tick = delta_time / (double)MICROTICKS_PER_TICK_fast;
                for (int i = 0; i <= (int)MICROTICKS_PER_TICK_fast - 1; i++)
                {
                    foreach (myRobot robot in robots)
                    {
                        Vector3 robot_action_target_velocity = new Vector3(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z);
                        Vector3 robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                        if (robot.touch)
                        {
                            Vector3 target_velocity = new Vector3(robot_action_target_velocity);
                            target_velocity.Clamp(ROBOT_MAX_GROUND_SPEED);
                            target_velocity -= new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) * dot(new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z), target_velocity);
                            Vector3 target_velocity_change = target_velocity - new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                            if (target_velocity_change.Length > 0)
                            {
                                double acceleration = ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal_y);
                                Vector3 target_velocity_change_tmp = new Vector3(target_velocity_change);
                                target_velocity_change_tmp.Norm();
                                target_velocity_change_tmp = target_velocity_change_tmp * acceleration * delta_time_tick;
                                target_velocity_change_tmp.Clamp(target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + target_velocity_change_tmp.x;
                                robot.velocity_y = robot.velocity_y + target_velocity_change_tmp.y;
                                robot.velocity_z = robot.velocity_z + target_velocity_change_tmp.z;
                            }
                        }
                        if (robot.action.use_nitro)
                        {
                            Vector3 target_velocity_change = new Vector3(robot_action_target_velocity - robot_velocity);
                            target_velocity_change.Clamp(robot.nitro_amout * NITRO_POINT_VELOCITY_CHANGE);
                            if (target_velocity_change.Length > 0)
                            {
                                Vector3 target_velocity_change_norm = new Vector3(target_velocity_change);
                                target_velocity_change_norm.Norm();
                                Vector3 acceleration = target_velocity_change_norm * ROBOT_NITRO_ACCELERATION;
                                Vector3 velocity_change = Clamp(acceleration * delta_time_tick, target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + target_velocity_change.x;
                                robot.velocity_y = robot.velocity_y + target_velocity_change.y;
                                robot.velocity_z = robot.velocity_z + target_velocity_change.z;
                                robot.nitro_amout -= velocity_change.Length / NITRO_POINT_VELOCITY_CHANGE;
                            }
                        }
                        robot.Move(delta_time_tick);
                        robot.radius = ROBOT_MIN_RADIUS + (Myrules.ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
                        robot.radius_change_speed = robot.action.jump_speed;
                    }

                    ball.Move(delta_time_tick);
                    ball.colliseArena();
                    for (int k = 0; k < robots.Length - 1; k++)
                        for (int j = 0; j < k; j++)
                        {
                            Entity a = new Entity(robots[k]);
                            Entity b = new Entity(robots[j]);
                            collide_entities(a, b);
                            robots[k].x = a.position.x; robots[k].y = a.position.y; robots[k].z = a.position.z;
                            robots[k].velocity_x = a.velocity.x; robots[k].velocity_y = a.velocity.y; robots[k].velocity_z = a.velocity.z;
                            robots[j].x = b.position.x; robots[j].y = b.position.y; robots[j].z = b.position.z;
                            robots[j].velocity_x = b.velocity.x; robots[j].velocity_y = b.velocity.y; robots[j].velocity_z = b.velocity.z;
                        }
                    foreach (myRobot robot in robots.Where(r => r.id == meR.id))
                    {
                        Entity a = new Entity(robot);
                        Entity b = new Entity(ball);
                        collide_entities(a, b);
                        robot.x = a.position.x; robot.y = a.position.y; robot.z = a.position.z;
                        robot.velocity_x = a.velocity.x; robot.velocity_y = a.velocity.y; robot.velocity_z = a.velocity.z;
                        ball.x = b.position.x; ball.y = b.position.y; ball.z = b.position.z;
                        ball.velocity_x = b.velocity.x;
                        ball.velocity_y = b.velocity.y;
                        ball.velocity_z = b.velocity.z;
                        robot.colliseArena();
                    }
                    if (ball.z > arena.depth / 2 + ball.radius)
                    {
                        break;
                    }
                    if (-ball.z < -arena.depth / 2 - ball.radius)
                    {
                        break;
                    }

                }
            }

        }

        public class Entity
        {
            public double x, y, z;
            public double velocity_x, velocity_y, velocity_z;
            public double mass;
            public double radius;
            public string type;
            public double radius_change_speed;
            public Vector3 position;
            public Vector3 velocity;
            public int id;
            public Entity(myBall var)
            {
                x = var.x;
                y = var.y;
                z = var.z;
                velocity_x = var.velocity_x;
                velocity_y = var.velocity_y;
                velocity_z = var.velocity_z;
                mass = BALL_MASS;
                radius = BALL_RADIUS;
                radius_change_speed = 0;
                position = new Vector3(x, y, z);
                velocity = new Vector3(velocity_x, velocity_y, velocity_z);
                type = "Ball";
            }
            public Entity(myRobot var)
            {
                x = var.x;
                y = var.y;
                z = var.z;
                velocity_x = var.velocity_x;
                velocity_y = var.velocity_y;
                velocity_z = var.velocity_z;
                mass = ROBOT_MASS;
                radius = var.radius;
                radius_change_speed = var.radius_change_speed;
                position = new Vector3(x, y, z);
                velocity = new Vector3(velocity_x, velocity_y, velocity_z);
                id = var.id;
                type = "Robot";
            }
        }

        public void Sim_ball(ref myBall ball)
        {
            myBall b = new myBall(ball);
            ball.MoveTick();
            if (ball.y < BALL_RADIUS)
            {
                Sim_ball_full(b);
                ball = b;
                return;
            }
            if (ball.colliseArena())
            {
                Sim_ball_full(b);
                ball = b;
                return;
            }
        }

        public void Sim_ball_full(myBall ball)
        {
            double delta_time = 1 / TICKS_PER_SECOND;
            double delta_time_tick = delta_time / (double)MICROTICKS_PER_TICK;
            for (int i = 0; i <= (int)MICROTICKS_PER_TICK - 1; i++)
            {
                ball.Move(delta_time / (double)MICROTICKS_PER_TICK);
                ball.colliseArena();
            }
        }


        public class BallWay
        {
            public myBall ball;
            public int tick;
            public int nearRid;
            public BallWay(myBall b, int tick)
            {
                this.tick = tick;
                ball = new myBall(b);
                nearRid = 0;
            }
        }


        public Vector3 getGoalVector(myRobot me, myBall ball)
        {
            Vector3 vec1 = new Vector3(0, 0, 42);
            Vector3 vec2 = new Vector3(30, 0, (42 - ball.z) * 0.5);
            Vector3 vec3 = new Vector3(-30, 0, (42 - ball.z) * 0.5);
            Vector3 toBall = new Vector3(ball.x - me.x, 0, ball.z - me.z);
            return vec1;

        }


        public double calcScoreGoalKeeper(myBall b, Game game)
        {
            double sc = b.velocity_y + b.velocity_z;
            for (int i = 0; i < 190; i++)
            {
                if (b.z > 0)
                    break;
                if (b.y > 17.6 && b.velocity_y > 0 && b.z < -30)
                    sc = sc - 600;
                if (b.y > 10 && b.z > -37)
                    sc = sc - 600;
                foreach (var enemy in game.robots.Where(r => r.is_teammate == false && r.z > b.x))
                {
                    int jump = 0;
                    if (enemy.nitro_amount > 0)
                        jump = getTickToJumpNitro(b.y);
                    else
                        jump = getFirstTickToJump(b.y);
                    double lenToBall = new Vector(enemy.x - b.x, enemy.z - b.z).LengthSq;
                    if (lenToBall < 4 * 4 && b.y < 3)
                        sc = sc - 10000;
                    if (lenToBall * 2 < i - jump)
                        sc = sc - 10;
                }
                Sim_ball(ref b);
            }

            if (b.z < 0)
                sc = sc - 100;
            return sc;
        }



        public List<myAction> getActionDirect(myRobot me, Game game, Vector3 posBall, int tick)
        {
            if (posBall.y > 10)
                return null;
            List<myAction> myActions = new List<myAction>();
            List<myAction> myActionsTmp = new List<myAction>();
            Vector3 vecToPos = new Vector3(posBall - me.Pos());
            myAction ma = null;
            vecToPos.y = 0;
            if (vecToPos.LengthSq > (0.5 * tick) * (0.5 * tick))
                return null;
            int jump = getFirstTickToJump(posBall.y) + 1;
            if (me.nitro_amout > 0)
                jump = getFirstTickJump(new myRobot(me), posBall.y) - 1;
            if (jump > tick)
                return null;
            int optimalTick = getOptimalTickJump(new myRobot(me), posBall.y);
            optimalTick = jump;
            myRobot rCalc = new myRobot(me);
            myBall bCalc = new myBall(game.ball);
            myActions = new List<myAction>();

            Vector3 vecToP = new Vector3(posBall - rCalc.Pos());
            vecToP.y = 0;
            double bestScore = 0;
            for (int k = jump; k < jump + 10; k++)
            {
                rCalc = new myRobot(me);
                int cntTick = game.current_tick;
                myActionsTmp = new List<myAction>();
                for (int i = 0; i <= tick + 5; i++)
                {
                    bCalc = new myBall(ballWays[i].ball);
                    if (i > 190)
                        break;
                    //                bCalc = new myBall(ballWays[i].ball);

                    if (rCalc.z > bCalc.z)
                        break;
                    vecToP = new Vector3(posBall - rCalc.Pos());
                    vecToP.y = 0;
                    double needSpeed = 0;
                    if (vecToP.Length > 1)
                    {
                        needSpeed = (vecToP.Length) * 60 / (tick - i - jump);

                        double myMaxSpeed = (tick - i - jump) * 1.6666666666;
                        if (myMaxSpeed > 30)
                            myMaxSpeed = 30;

                        if (tick - i - jump <= 0)
                            needSpeed = 30;
                        if (needSpeed > 30 && (tick - i - jump) > 0 && vecToP.LengthSq > 9)
                            break;
                        double curSpeed = rCalc.SpeedVec().Length;
                        if (myMaxSpeed - 5 > needSpeed && curSpeed < 0.2 && needSpeed > 2)
                            needSpeed = 0;


                        rCalc.action.use_nitro = false;
                        if (needSpeed - curSpeed > 5 && needSpeed > 0)
                            rCalc.action.use_nitro = true;
                    }

                    int tickToTouch = tick - i;

                    vecToP.Norm();
                    vecToP = vecToP * needSpeed;
                    rCalc.action.target_velocity_x = vecToP.x;
                    rCalc.action.target_velocity_y = 0;
                    rCalc.action.target_velocity_z = vecToP.z;
                    if (rCalc.z < -41 && Math.Abs(rCalc.x) > 13)
                    {
                        rCalc.action.target_velocity_x = 0;
                        rCalc.action.target_velocity_y = 0;
                        rCalc.action.target_velocity_z = 30;
                    }

                    if (tickToTouch <= k)
                    {
                        rCalc.action.target_velocity_x = vecToP.x;
                        rCalc.action.target_velocity_y = 30;
                        rCalc.action.target_velocity_z = vecToP.z;
                        rCalc.action.use_nitro = true;
                        rCalc.action.jump_speed = 15;
                    }

                    ma = new myAction(rCalc.action, cntTick);
                    ma.r = new myRobot(rCalc);
                    ma.ball = new myBall(bCalc);
                    myActionsTmp.Add(ma);
                    cntTick++;
                    isTouchBall = false;
                    Update(ref rCalc, ref bCalc, true);
                    if (bCalc.z < -42)
                        break;
                    //                    double lenTest = (rCalc.Pos() - bCalc.Pos()).Length;

                    if (isTouchBall && bCalc.velocity_z < 0 && myActions.Count == 0)
                    {
                        if (ballWays.FirstOrDefault(bw => bw.ball.z <= -42) != null)
                        {
                            if (!isGoalMne(bCalc))
                            {
                                myActions = myActionsTmp;
                                break;
                            }
                        }

                    }

                    if (isTouchBall && bCalc.velocity_z > 0)
                    {
                        if (myActions.Count == 0)
                            myActions = myActionsTmp;
                        double sc = calcScoreGoalKeeper(new myBall(bCalc), game);
                        if (bestScore == 0 || sc > bestScore)
                        {
                            bestScore = sc;
                            myActions = myActionsTmp;
                            break;
                        }
                    }

                    if (isTouchBall && isGoalMne(new myBall(bCalc)))
                        break;
                }
            }

            if (myActions.Count > 0)
                return myActions;
            return null;
        }

        public bool isEnemyFirst(myRobot me, Vector3 BallPos, Game game)
        {
            double len = (BallPos - me.Pos()).LengthSq;
            foreach (var r in game.robots)
            {
                if (r.is_teammate)
                    continue;
                if (r.z > BallPos.z && (BallPos - new Vector3(r.x, r.y, r.z)).LengthSq < len)
                    return true;
            }
            return false;
        }

        public Model.Action Defender(Robot me, Game game)
        {
            Model.Action a = null;
            actionList = new List<myAction>();
            List<myAction> mTmp = new List<myAction>();
            for (int i = 0; i < 100; i++)
            {
                if (ballWays[i].ball.y > 9)
                    continue;
                if (ballWays[i].ball.z > -15)
                    continue;
                myBall bGoal = new myBall(ballWays[i].ball);
                if (Math.Abs(ballWays[i].ball.x) < 14)
                {
                    if (bGoal.z > -35)
                    {
                        mTmp = getActionDirect(new myRobot(me), game, bGoal.Pos(), i);
                        if (mTmp != null)
                        {
                            actionList = mTmp;
                            a = new Model.Action();
                            a.target_velocity_x = actionList[0].action.target_velocity_x;
                            a.target_velocity_y = actionList[0].action.target_velocity_y;
                            a.target_velocity_z = actionList[0].action.target_velocity_z;
                            a.jump_speed = actionList[0].action.jump_speed;
                            a.use_nitro = actionList[0].action.use_nitro;
                            return a;
                        }
                    }

                    if (bGoal.z < -35)
                    {
                        int tickToTouch = 0;
                        myRobot rCalc = new myRobot(me);
                        for (int p = 0; p < i; p++)
                        {
                            rCalc = new myRobot(me);
                            actionList = new List<myAction>();
                            int tick = game.current_tick;
                            for (int j = 0; j < p; j++)
                            {
                                rCalc.action.target_velocity_x = 0;
                                rCalc.action.target_velocity_y = 0;
                                rCalc.action.target_velocity_z = 0;
                                myAction ma = new myAction(rCalc.action, tick);
                                ma.ball = new myBall(ballWays[tick - game.current_tick].ball);
                                ma.r = new myRobot(rCalc);
                                actionList.Add(ma);
                                tick++;
                                Sim_Robot_tick(ref rCalc);
                            }

                            for (int x = (int)(bGoal.x - 5); x < (int)(bGoal.x + 5); x++)
                            {
                                if (tick - game.current_tick > i)
                                    break;
                                if (rCalc.touch_normal_y == 1)
                                {
                                    Vector3 vel = MoveToVec(rCalc, x, 0, -50);
                                    rCalc.action.target_velocity_x = vel.x;
                                    rCalc.action.target_velocity_z = vel.z;
                                }
                                else
                                {
                                    rCalc.action.target_velocity_x = 0;
                                    rCalc.action.target_velocity_z = -30;
                                    rCalc.action.target_velocity_y = 30;
                                }

                                myAction ma = new myAction(rCalc.action, tick);
                                ma.ball = new myBall(ballWays[tick - game.current_tick].ball);
                                ma.r = new myRobot(rCalc);
                                actionList.Add(ma);
                                tick++;
                                Sim_Robot(ref rCalc);
                            }

                            double len = (rCalc.Pos() - bGoal.Pos()).LengthSq;
                            if (len < 9 && actionList.Count() > 0)
                            {
                                a.target_velocity_x = actionList[0].action.target_velocity_x;
                                a.target_velocity_y = actionList[0].action.target_velocity_y;
                                a.target_velocity_z = actionList[0].action.target_velocity_z;
                                return a;
                            }
                        }
                    }
                }

            }

            return null;
        }


        public Model.Action SaveGoal(int tickGoal, Robot me, Game game)
        {
            if (tickGoal > 100)
                return null;

            Model.Action a = null;
            myRobot rMain = new myRobot(me);
            myAction ma = null;
            if (actionList == null)
                actionList = new List<myAction>();
            ClearActionList(rMain.id);
            List<myAction> actionListTmp = new List<myAction>();
            myBall bestBall = null;
            bool isGoalbestBall = false;
            for (int i = 0; i < tickGoal; i++)
            {
                actionListTmp = new List<myAction>();
                bool isNitroJump = false;
                bool isCanJump = false;
                Vector3 posBall = ballWays[i].ball.Pos();
                posBall.z = posBall.z - 2;

                if (posBall.y > 10)
                    continue;
                if (Math.Abs(posBall.x) > 24 && posBall.z < -38)
                    continue;
                Vector3 toBallGround = new Vector3(posBall.x - rMain.x, 0, posBall.z - rMain.z);
                double len = toBallGround.LengthSq;
                if (len * 4 > i * i)
                    continue;
                if (isEnemyFirst(new myRobot(me), ballWays[i].ball.Pos(), game) && posBall.z > -30)
                    continue;
                if (ballWays[i].ball.z < -42)
                    return null;


                int jump = 0;
                jump = getFirstTickToJump(posBall.y);
                if (meR.nitro_amount > 0)
                {
                    jump = getFirstTickJump(new myRobot(me), posBall.y);
                }
                List<myAction> mTmp = getActionDirect(new myRobot(rMain), game, posBall, i);
                if (mTmp != null)
                {
                    actionList = mTmp;
                    a = new Model.Action();
                    a.target_velocity_x = actionList[0].action.target_velocity_x;
                    a.target_velocity_y = actionList[0].action.target_velocity_y;
                    a.target_velocity_z = actionList[0].action.target_velocity_z;
                    a.jump_speed = actionList[0].action.jump_speed;
                    a.use_nitro = actionList[0].action.use_nitro;
                    return a;
                }
            }

            if (actionList.Count > 0)
            {
                ma = actionList.FirstOrDefault(m => m.tick == game.current_tick && m.r.id == rMain.id);
                if (ma != null)
                {
                    a = new Model.Action();
                    a.target_velocity_x = ma.action.target_velocity_x;
                    a.target_velocity_y = ma.action.target_velocity_y;
                    a.target_velocity_z = ma.action.target_velocity_z;
                    a.jump_speed = ma.action.jump_speed;
                    a.use_nitro = ma.action.use_nitro;
                    return a;
                }
            }
            return null;
        }


        public List<Model.Action> getWayToPosFast(myRobot me, Vector3 BallPos, Vector3 VelVec, int tikeTime, int jumpDiff, Vector3 BallPosTouch, int endSpeed = 5)
        {
            int jump = 1;
            if (BallPos.y < 1)
                BallPos.y = 1;

            List<Model.Action> aList = new List<Model.Action>();


            VelVec.Norm();
            double beginLen = new Vector(me.x - BallPos.x, me.z - BallPos.z).LengthSq;
            if (beginLen > (tikeTime * 0.5 * tikeTime * 0.5))
                return null;
            double speed = 30;
            double checkX, checkZ;
            int testTick;
            if (tikeTime < 0)
                return null;
            //            if (me.nitro_amout == 0)
            //                jump = getFirstTickJump(new myRobot(me), BallPos.y);
            //            if (me.nitro_amout > 0)
            //                jump = getOptimalTickJump(new myRobot(me), BallPos.y) - 1;
            jump = getFirstTickJump(new myRobot(me), BallPos.y);
            jump = jump + jumpDiff;
            if (jump < 0)
                return null;
            tikeTime = tikeTime - jump;
            Vector3 curVecSpeed = new Vector3(me.velocity_x, 0, me.velocity_z);
            for (int i = 0; i < 17; i++)
            {
                testTick = 0;
                myRobot calcR = new myRobot(me);
                myRobot meR = new myRobot(me);
                Vector3 speedR = VelVec * speed;
                calcR.velocity_x = -speedR.x;
                calcR.velocity_z = -speedR.z;
                calcR.x = BallPos.x - speedR.x / 60 * jump;
                calcR.z = BallPos.z - speedR.z / 60 * jump;
                checkX = calcR.x;
                checkZ = calcR.z;
                if (checkZ < -47)
                    continue;
                aList = new List<Model.Action>();
                List<Vector3> listPos = new List<Vector3>();
                List<myRobot> listR = new List<myRobot>();
                //ïðûæîê
                for (int j = 0; j < jump; j++)
                {
                    calcR.action.use_nitro = true;
                    calcR.action.target_velocity_y = 30;
                    calcR.action.jump_speed = 15;
                    calcR.action.target_velocity_x = calcR.velocity_x;
                    calcR.action.target_velocity_z = calcR.velocity_z;
                    listR.Add(new myRobot(calcR));
                }

                calcR.action.jump_speed = 0;
                calcR.action.use_nitro = false;
                calcR.action.target_velocity_y = 0;
                int cntWhile = 0;
                while (testTick * 2 < tikeTime)
                {
                    cntWhile++;
                    if (cntWhile > 150)
                        break;
                    int toEndTick = tikeTime - testTick * 2;
                    double len = new Vector(meR.x - calcR.x, meR.z - calcR.z).Length;
                    double needSpeed = 60 * len / (toEndTick);
                    if (toEndTick == 0)
                        needSpeed = 30;
                    if (needSpeed > ROBOT_MAX_GROUND_SPEED + 1 && len > 3)
                        break;
                    Vector3 toCalcR = new Vector3(-meR.x + calcR.x, 0, -meR.z + calcR.z);
                    toCalcR.Norm();
                    toCalcR = toCalcR * needSpeed;
                    Model.Action a = new Model.Action();
                    calcR.action.target_velocity_x = -toCalcR.x;
                    calcR.action.target_velocity_z = -toCalcR.z;
                    meR.action.target_velocity_x = toCalcR.x;
                    meR.action.target_velocity_z = toCalcR.z;

                    if (meR.z < -41 && Math.Abs(meR.x) > 10)
                    {
                        meR.action.target_velocity_x = 0;
                        meR.action.target_velocity_y = 0;
                        meR.action.target_velocity_z = 30;
                    }

                    double curSpeed = new Vector(calcR.velocity_x, calcR.velocity_z).Length;
                    Vector mSpeed = new Vector(meR.velocity_x, meR.velocity_z);
                    mSpeed.Norm();
                    Vector calcSpeed = new Vector(calcR.velocity_x, calcR.velocity_z);
                    calcSpeed.Norm();
                    Vector diffSpeed = mSpeed + calcSpeed;
                    //                    if (Math.Abs(meR.velocity_x - toCalcR.x) < 0.01 && Math.Abs(meR.velocity_z - toCalcR.z) < 0.01 && Math.Abs(calcR.velocity_x - toMainR.x) < 0.01 && Math.Abs(calcR.velocity_z - toMainR.z) < 0.01 && Math.Abs(meR.velocity_x + calcR.velocity_x) < 0.01 && Math.Abs(meR.velocity_z + calcR.velocity_z) < 0.01)
                    if (Math.Abs(diffSpeed.x) < 0.001 && Math.Abs(diffSpeed.y) < 0.001)
                    {

                        Vector3 vel = new Vector3(calcR.Pos() - meR.Pos());
                        vel.Norm();
                        for (int j = 0; j < toEndTick; j++)
                        {
                            len = new Vector(meR.x - calcR.x, meR.z - calcR.z).Length;
                            if (tikeTime - testTick * 2 - j != 0)
                                needSpeed = 60 * len / (toEndTick - j);
                            if (toEndTick - j == 0)
                                needSpeed = 30;
                            if (needSpeed > ROBOT_MAX_GROUND_SPEED && len > 3)
                                break;
                            double curSpeedMe = new Vector(meR.velocity_x, meR.velocity_z).Length;
                            int tickForNeedSpeed = (int)(Math.Abs(curSpeed - curSpeedMe) / 1.666666) + 1;
                            if (tickForNeedSpeed >= toEndTick - j)
                                needSpeed = curSpeed;
                            meR.action.target_velocity_x = vel.x * needSpeed;
                            meR.action.target_velocity_z = vel.z * needSpeed;
                            a = new Model.Action
                            {
                                target_velocity_x = meR.action.target_velocity_x,
                                target_velocity_z = meR.action.target_velocity_z
                            };
                            aList.Add(a);
                            Sim_Robot_tick(ref meR);
                        }
                        len = new Vector(meR.x - calcR.x, meR.z - calcR.z).Length;
                        if (len > 1)
                            break;
                        if (needSpeed > ROBOT_MAX_GROUND_SPEED)
                            break;
                        for (int j = listR.Count - 1; j >= 0; j--)
                        {
                            a = new Model.Action();
                            //toCalcR = MoveToVec(meR, BallPos.x, 0, BallPos.z);
                            toCalcR = new Vector3(-listR[j].velocity_x, 0, -listR[j].velocity_z);
                            meR.action.target_velocity_x = toCalcR.x;
                            meR.action.target_velocity_y = listR[j].action.target_velocity_y;
                            meR.action.target_velocity_z = toCalcR.z;
                            meR.action.use_nitro = listR[j].action.use_nitro;
                            meR.action.jump_speed = listR[j].action.jump_speed;
                            a.target_velocity_x = meR.action.target_velocity_x;
                            a.target_velocity_z = meR.action.target_velocity_z;
                            a.target_velocity_y = meR.action.target_velocity_y;
                            a.use_nitro = meR.action.use_nitro;
                            a.jump_speed = meR.action.jump_speed;
                            aList.Add(a);
                            Sim_Robot_tick(ref meR);
                        }

                        if ((meR.Pos() - BallPosTouch).LengthSq > 9)
                            break;
                        return aList;
                    }
                    a.target_velocity_x = meR.action.target_velocity_x;
                    a.target_velocity_z = meR.action.target_velocity_z;
                    aList.Add(a);
                    listPos.Add(new Vector3(calcR.x, 0, calcR.z));
                    listR.Add(new myRobot(calcR));
                    Sim_Robot_tick(ref meR);
                    Sim_Robot_tick(ref calcR);
                    testTick++;

                }
                speed = speed - 1.666666666666666;
            }


            return null;
        }

        public Model.Action CorrectJump(Robot me, Game game)
        {
            Model.Action a = null;
            myRobot rCalc = new myRobot(me);
            double bestScore = 0;
            for (int jumpSpeed = 1; jumpSpeed <= 15; jumpSpeed++)
            {
                myBall cBall = new myBall(game.ball);
                rCalc = new myRobot(me);
                for (int i = 0; i < 50; i++)
                {
                    isTouchBall = false;
                    rCalc.action.jump_speed = jumpSpeed;
                    rCalc.action.target_velocity_y = 30;
                    rCalc.action.use_nitro = true;
                    cBall = new myBall(ballWays[i].ball);
                    Update(ref rCalc, ref cBall);

                    if (isTouchBall)
                        break;
                    if (rCalc.touch)
                        break;
                }
                if (isTouchBall)
                {
                    double sc = calcScore(cBall, game);
                    if (bestScore == 0 || sc > bestScore)
                    {
                        bestScore = sc;
                        a = rCalc.action;
                    }
                }
            }

            return a;
        }

        public void Act(Robot me, Rules rules, Game game, Model.Action action)
        {

            AVG = timeCPU / (game.current_tick + 1);
            //AVG = 11;
            Stopwatch sw = Stopwatch.StartNew();
            Model.Action a = new Model.Action();


            Act_(me, rules, game, out a);

            if (Double.IsInfinity(a.target_velocity_x) || Double.IsInfinity(a.target_velocity_z))
                Console.WriteLine("Infinity");

            if (Double.IsInfinity(a.target_velocity_x))
                a.target_velocity_x = 30;
            if (Double.IsInfinity(a.target_velocity_y))
                a.target_velocity_y = 30;
            if (Double.IsInfinity(a.target_velocity_z))
                a.target_velocity_z = 30;


            action.jump_speed = a.jump_speed;
            action.target_velocity_x = a.target_velocity_x;
            action.target_velocity_y = a.target_velocity_y;
            action.target_velocity_z = a.target_velocity_z;
            action.use_nitro = a.use_nitro;
            sw.Stop();
            timeCPU = timeCPU + sw.Elapsed.TotalMilliseconds + 0.8;
            if (sw.Elapsed.TotalMilliseconds > 4000)
                Console.WriteLine("timeCPU=" + sw.Elapsed.TotalMilliseconds + " tick=" + game.current_tick);
            if (game.current_tick >= rules.max_tick_count - 1)
                Console.WriteLine("timeCPU=" + timeCPU + " AVG=" + timeCPU / game.current_tick);

            if (game.current_tick % 1000 == 0 && (me.id == 1 || me.id == 4))
                Console.WriteLine("timeCPU=" + timeCPU + " AVG=" + timeCPU / (game.current_tick + 1));
        }

        public void Act_(Robot me, Rules rules, Game game, out Model.Action action)
        {
            action = new Model.Action();
            int lastTickAction = 0;
            if (calcMoves == null || calcMoves.Count == 0)
                initVec();

            if (actionList != null)
                if (actionList.Count >= 0)
                {
                    if (game.robots.Length == 4)
                    {
                        if (lastRobot == 1 || lastRobot == 3)
                            actionList1 = actionList;
                        if (lastRobot == 2 || lastRobot == 4)
                            actionList2 = actionList;
                    }
                    if (game.robots.Length == 6)
                    {
                        if (lastRobot == 1 || lastRobot == 4)
                            actionList1 = actionList;
                        if (lastRobot == 2 || lastRobot == 5)
                            actionList2 = actionList;
                        if (lastRobot == 3 || lastRobot == 6)
                            actionList3 = actionList;
                    }
                }

            if (game.robots.Length == 4)
            {
                if (me.id == 1 || me.id == 3)
                {
                    actionList = actionList1;

                }

                if (me.id == 2 || me.id == 4)
                    actionList = actionList2;
            }

            if (game.robots.Length == 6)
            {
                if (me.id == 1 || me.id == 4)
                    actionList = actionList1;

                if (me.id == 2 || me.id == 5)
                    actionList = actionList2;

                if (me.id == 3 || me.id == 6)
                    actionList = actionList3;
            }

            lastRobot = me.id;
            int nearBallTouch = -1;
            double eps = 0.1;
            Robot r1, r2, r3;
            myAction ma = null;
            Vector3 ballPos;
            Vector3 posBall;
            Vector3 vecToGoal;
            Vector3 velToBall;
            Vector3 bestVelToBall;

            arena = rules.arena;
            Myrules = rules;
            meR = me;

            r1 = me;
            r2 = me;
            r3 = null;
            foreach (Robot r in game.robots)
                if (r.is_teammate && r.id != me.id)
                {
                    if (r2.id == me.id)
                        r2 = r;
                    else
                        r3 = r;
                }

            if (r3 == null)
                r3 = r2;

            if (game.players.Length > 1)
            {
                if (game.players[0].score != score1 || game.players[1].score != score2)
                {
                    actionList = null;
                    actionList1 = null;
                    actionList2 = null;
                    actionList3 = null;
                    //Console.WriteLine(game.players[0].score + ":" + game.players[1].score);
                }
                score1 = game.players[0].score;
                score2 = game.players[1].score;
            }

            if (game.ball.z < -rules.arena.depth * 0.5 - BALL_RADIUS)
                return;
            if (game.ball.z > rules.arena.depth * 0.5 + BALL_RADIUS)
                return;
            /*
                        if (me.id != 3)
                                return;
                        if (game.current_tick < 10)
                        {
                            action.target_velocity_x = 30;
                            action.target_velocity_z = 20;
                            return;
                        }

                        action.target_velocity_x = 0;
                        action.target_velocity_z = 0;
                        action.jump_speed = 15;
                        action.use_nitro = true;
                        return;


                        if (game.current_tick < 200)
                                                            {
                                                                Vector3 vtest = MoveTo(me, 0, 0, -43.5);
                                                                action.target_velocity_x = vtest.x;
                                                                action.target_velocity_z = vtest.z;
                                                                return;
                                    }

                                                            myRobot robotCalc=  new myRobot(me);
                                                            robotCalc.x = 0;
                                                            robotCalc.z = -43.5;
                                                            robotCalc.velocity_x = 0;
                                                            robotCalc.velocity_z = 0;
                                    StreamWriter rw = new StreamWriter("robot.txt",false);
                                    for (int jump = 180; jump <= 180; jump++)
                                    {
                                        robotCalc = new myRobot(me);
                                        robotCalc.x = 0;
                                        robotCalc.z = -43.5;
                                        robotCalc.velocity_x = 0;
                                        robotCalc.velocity_z = 0;
                                        rw.WriteLine("");
                                        for (int i = 0; i < 100; i++)
                                        {
                                            robotCalc.action.jump_speed = 0;
                                            if (i == jump)
                                                robotCalc.action.jump_speed = 15;
                                            robotCalc.action.target_velocity_x = 30;
                                            robotCalc.action.target_velocity_z = 30;
                                            robotCalc.action.target_velocity_y = 30;

                                            if (i < 35)
                                            {
                                                robotCalc.action.target_velocity_x = 30;
                                                robotCalc.action.target_velocity_z = -2;
                                                robotCalc.action.target_velocity_y = 30;
                                            }

                                            if (robotCalc.z>-42)
                                                robotCalc.action.target_velocity_z = 30;

                                            if (Math.Abs(robotCalc.x) < ESP)
                                                robotCalc.x = 0;
                                            calcMoves.Add(new calcMove(i, robotCalc.x, robotCalc.y, robotCalc.z,
                                                robotCalc.action.target_velocity_x, robotCalc.action.target_velocity_y,
                                                robotCalc.action.target_velocity_z, robotCalc.action.jump_speed,
                                                robotCalc.action.use_nitro));
                                            string res = String.Format(
                                                "calcMoves.Add(new calcMove({0}, {1}, {2},{3},{4},{5},{6},{7},{8}));", i,
                                                robotCalc.x.ToString().Replace(',', '.'), robotCalc.y.ToString().Replace(',', '.'),
                                                robotCalc.z.ToString().Replace(',', '.'),
                                                robotCalc.action.target_velocity_x.ToString().Replace(',', '.'),
                                                robotCalc.action.target_velocity_y.ToString().Replace(',', '.'),
                                                robotCalc.action.target_velocity_z.ToString().Replace(',', '.'),
                                                robotCalc.action.jump_speed.ToString().Replace(',', '.'),
                                                robotCalc.action.use_nitro.ToString().ToLower());
                                            rw.WriteLine(res);
                                            Sim_Robot(ref robotCalc);
                                        }
                                    }

                                    rw.Close();
                                                            action.target_velocity_x = 30;
                                                            action.target_velocity_z = -30;
                                                            action.target_velocity_y = 30;
                                                            if (me.y > 20)
                                                                action.jump_speed = 15;
                                                            if (game.current_tick >280)
                                                                action.target_velocity_y = 30;
                                    if (rTeset != null)
                                                                Console.WriteLine((game.current_tick-200).ToString() + " " + (me.y) + " " + me.z + " " + me.velocity_z);
                                                            rTeset = new myRobot(me);
                                                            rTeset.action.target_velocity_z = action.target_velocity_z;
                                                            rTeset.action.target_velocity_x = action.target_velocity_x;
                                                            rTeset.action.target_velocity_y = action.target_velocity_y;
                                                            rTeset.action.jump_speed = action.jump_speed;
                                                            Sim_Robot(ref rTeset);
                                                            return;

                */

            if (game.current_tick >= 1170 && me.id == 5)
                nearBallTouch = -1;

            CalcBallWays(game);

            bool isDefender = false;

            bool isRecalc = false;
            if (actionList != null && actionList.Count > 0)
            {
                ma = actionList.FirstOrDefault(m => m.tick == game.current_tick && m.r.id == me.id);
                if (ma != null)
                {
                    if (Math.Abs(ma.r.x - me.x) > eps || Math.Abs(ma.r.z - me.z) > eps || Math.Abs(ma.ball.y - game.ball.y) > eps || Math.Abs(ma.ball.z - game.ball.z) > eps || Math.Abs(ma.ball.x - game.ball.x) > eps)
                    {
                        if (me.touch_normal_y == 1 && me.touch)
                        {
                            isRecalc = true;
                            if (firstTick > 0)
                            {
                                actionList = new List<myAction>();
                                actionList1 = new List<myAction>();
                                actionList2 = new List<myAction>();
                                actionList3 = new List<myAction>();
                                CalcBallWays(game);
                            }
                        }
                    }

                    if (!isRecalc)
                    {
                        action.target_velocity_x = ma.action.target_velocity_x;
                        action.target_velocity_y = ma.action.target_velocity_y;
                        action.target_velocity_z = ma.action.target_velocity_z;
                        action.jump_speed = ma.action.jump_speed;
                        action.use_nitro = ma.action.use_nitro;
                        if (action.jump_speed > 0)
                            action.use_nitro = true;
                        myRobot rTestTouch = new myRobot(me);
                        myBall bTestTouch = new myBall(game.ball);
                        rTestTouch.action.jump_speed = 15;
                        rTestTouch.action.use_nitro = true;
                        rTestTouch.action.target_velocity_y = 30;
                        isTouchBall = false;
                        Update(ref rTestTouch, ref bTestTouch, true);
                        if (isTouchBall)
                        {
                            action.jump_speed = 15;

                                for (int j = 15; j >= 1; j--)
                                {
                                    bTestTouch = new myBall(game.ball);
                                    rTestTouch = new myRobot(me);
                                    rTestTouch.action.jump_speed = j;
                                    rTestTouch.action.use_nitro = true;
                                    rTestTouch.action.target_velocity_y = 30;
                                    isTouchBall = false;
                                    Update(ref rTestTouch, ref bTestTouch, true);
                                if (isTouchBall == false)
                                    break;
                                    if (bTestTouch.velocity_y <= 0)
                                        break;
                                    while (bTestTouch.velocity_y > 0)
                                    {
                                        bTestTouch.MoveTick();
                                        if (bTestTouch.z > 37 && bTestTouch.y > 7 )
                                            break;
                                    }
                                    if (bTestTouch.z > 35 && bTestTouch.y > 7)
                                        continue;
                                    action.jump_speed = j;
                                    break;
                                }
                            action.use_nitro = true;
                            return;
                        }

                        return;
                    }
                }
            }

            if (me.touch == false)
            {
                Vector3 toBall = new Vector3(game.ball.x, game.ball.y, game.ball.z) - new Vector3(me.x, me.y, me.z);
                if (me.z < game.ball.z && toBall.LengthSq < 25 * 25)
                {

                    myRobot rTestTouch = new myRobot(me);
                    myBall bTestTouch = new myBall(game.ball);
                    isTouchBall = false;
                    for (int i = 0; i < 20; i++)
                    {
                        if (rTestTouch.touch)
                            break;
                        rTestTouch.action.jump_speed = 15;
                        rTestTouch.action.use_nitro = true;
                        rTestTouch.action.target_velocity_y = 30;
                        Update(ref rTestTouch, ref bTestTouch, true);
                        if (isTouchBall)
                        {
                            action.jump_speed = 1;
                            if (i==0)
                            {
                                
                                for (int j=15;j>=1;j--)
                                {
                                    bTestTouch = new myBall(game.ball);
                                    rTestTouch = new myRobot(me);
                                    rTestTouch.action.jump_speed = j;
                                    rTestTouch.action.use_nitro = true;
                                    rTestTouch.action.target_velocity_y = 30;
                                    Update(ref rTestTouch, ref bTestTouch, true);
                                    if (bTestTouch.velocity_y <= 0)
                                        break;
                                    while (bTestTouch.velocity_y>0)
                                    {
                                        bTestTouch.MoveTick();
                                        if (bTestTouch.z > 35 && bTestTouch.y > 8 || bTestTouch.y>17)
                                            break;
                                    }
                                    if (bTestTouch.z > 35 && bTestTouch.y > 8 || bTestTouch.y > 17)
                                        continue;
                                    action.jump_speed = j;
                                    break;
                                }
                            }
                            rTestTouch.action.target_velocity_y = 30;
                            action.use_nitro = true;
                            return;
                        }
                    }
                }

                return;
            }
            ClearActionList(me.id);


            if (me.touch_normal_y != 1)
            {
                Vector3 toBall = MoveToVec(me, game.ball.x, game.ball.y, game.ball.z);
                action.target_velocity_x = toBall.x;
                action.target_velocity_z = toBall.z;
                action.target_velocity_y = toBall.y;
                return;
            }


            if (me.z < r2.z && me.z < r3.z)
                isDefender = true;

            double curScore = calcScore(new myBall(game.ball), game);

            BallWay isMyGoal = ballWays.FirstOrDefault(bw => bw.ball.z >= 42);
            if (game.robots.Length == 6 && me.z > r2.z && me.z > r3.z && me.z > game.ball.z && isMyGoal != null)
            {
                double enemyZ = game.robots.Where(e => e.is_teammate == false).Max(ez => ez.z);
                myRobot enemy = new myRobot(game.robots.Where(e => e.is_teammate == false).FirstOrDefault(ez => ez.z >= enemyZ));
                if (enemy != null)
                {
                    Vector3 toEnemy = new Vector3(enemy.x + enemy.velocity_x / 60, 0, enemy.z + enemy.velocity_z / 60);
                    Vector3 toEnemyTouch = MoveToVec(me, toEnemy.x, 0, toEnemy.z - 1);
                    action.target_velocity_x = toEnemyTouch.x;
                    action.target_velocity_z = toEnemyTouch.z;
                    if (toEnemy.Length < 3)
                        action.jump_speed = 15;
                    return;
                }
            }

            double myLenBall, r2LenBall, r3LenBall;
            myLenBall = new Vector3(me.x - game.ball.x, 0, me.z - game.ball.z).LengthSq;
            r2LenBall = new Vector3(r2.x - game.ball.x, 0, r2.z - game.ball.z).LengthSq;
            r3LenBall = new Vector3(r3.x - game.ball.x, 0, r3.z - game.ball.z).LengthSq;

            if (myLenBall < r2LenBall && myLenBall < r3LenBall && game.ball.z < -30 && game.ball.velocity_z < 0 && me.z < game.ball.z)
                isDefender = false;

            double minBallz = ballWays.Min(bw => bw.ball.z);
            BallWay isMneGoal = ballWays.FirstOrDefault(bw => bw.ball.z <= -rules.arena.depth * 0.5 - 2);
            if (isMneGoal == null)
                isMneGoal = ballWays.FirstOrDefault(bw => bw.ball.z <= -40 && Math.Abs(bw.ball.x) < 17 && bw.ball.y < 8);
            double tickTomyGoal = (game.ball.z + 42) / (-game.ball.velocity_z / 60);
            if (game.robots.Length == 4 && isMneGoal == null && tickTomyGoal < 100 && game.ball.z < 0 && game.ball.velocity_z < 0 && r2.z > game.ball.z - 5 && r3.velocity_z < 0 && r3.z > game.ball.z - 5 && r3.velocity_z < 0)
                isDefender = false;


            List<myAction> actionSave = new List<myAction>();
            if (isMneGoal != null && isMneGoal.tick - game.current_tick < 100)
            {

                if (true || me.z < game.ball.z)
                {

                    Model.Action a = SaveGoal(isMneGoal.tick - game.current_tick, me, game);
                    if (a != null)
                    {
                        actionSave = actionList;
                        if (actionList.Count > 0)
                        {
                            action.target_velocity_x = a.target_velocity_x;
                            action.target_velocity_z = a.target_velocity_z;
                            action.target_velocity_y = a.target_velocity_y;
                            action.use_nitro = a.use_nitro;
                            action.jump_speed = a.jump_speed;
                            return;
                        }
                    }

                    if (a == null && me.z < r2.z && me.z < r3.z)
                    {
                        Vector3 vec = MoveTo(me, game.ball.x, 0, -42);
                        action.target_velocity_x = vec.x;
                        action.target_velocity_y = 0;
                        action.target_velocity_z = vec.z;
                        return;
                    }
                }
            }


            if (isDefender && isMneGoal == null)
            {
                if (isMneGoal == null && game.ball.z > -20 && game.ball.velocity_z > 0 && me.nitro_amount < 20 && game.nitro_packs.Length > 0)
                {
                    NitroPack n = null;
                    foreach (var np in game.nitro_packs)
                    {
                        if (np.z > 0)
                            continue;
                        if (n == null)
                            n = np;
                        if (Math.Abs(n.x - me.z) > Math.Abs(np.x - me.z))
                            n = np;
                    }

                    if (n != null)
                    {
                        Vector3 vecNp = MoveTo(me, n.x, 0, n.z);
                        action.target_velocity_x = vecNp.x;
                        action.target_velocity_y = 0;
                        action.target_velocity_z = vecNp.z;
                        return;
                    }
                }


                double dx = 0;
                if (game.ball.x > 15)
                    dx = 5;
                if (game.ball.x < -15)
                    dx = -5;
                Vector3 vec = MoveTo(me, dx, 0, -42);
                action.target_velocity_x = vec.x;
                action.target_velocity_y = 0;
                action.target_velocity_z = vec.z;
                return;
            }





            if (isStartGame(game))
            {
                Model.Action a = getStartWay(me, game);
                action.use_nitro = a.use_nitro;
                action.jump_speed = a.jump_speed;
                action.target_velocity_x = a.target_velocity_x;
                action.target_velocity_y = a.target_velocity_y;
                action.target_velocity_z = a.target_velocity_z;
                return;

            }






            nearBallTouch = 0;
            for (int i = 0; i < calcCount; i++)
            {
                if (ballWays[i].ball.z > 42)
                {
                    Vector3 vec = MoveTo(me, game.ball.x, 0, game.ball.z - 6);
                    action.target_velocity_x = vec.x;
                    action.target_velocity_y = vec.y;
                    action.target_velocity_z = vec.z;
                    break;
                }
                ballPos = new Vector3(ballWays[i].ball.x, ballWays[i].ball.y, ballWays[i].ball.z);
                double len = new Vector3(ballWays[i].ball.x - me.x, ballWays[i].ball.y - me.y, ballWays[i].ball.z - me.z).Length;
                if (len < 10)
                {
                    nearBallTouch = 0;
                    break;
                }
                int tickToBall = (int)(len * 2);
                if (i < tickToBall)
                    continue;
                nearBallTouch = i;
                break;
            }


            if (me.z > game.ball.z && firstTick == 0 && game.ball.velocity_z < 0)
            {
                Robot enemy = null;
                foreach (Robot e in game.robots.Where(r => r.is_teammate == false && r.touch))
                {
                    if (e.z > game.ball.z)
                    {
                        if (enemy == null)
                            enemy = e;
                        if (e.z < enemy.z)
                            enemy = e;
                    }
                }
                if (enemy != null && game.robots.Length == 6 && me.z > r2.z && me.z > r3.z)
                {
                    Vector3 toEnemy = new Vector3(enemy.x + enemy.velocity_x / 60 - me.x, 0, enemy.z + enemy.velocity_z / 60 - me.z);
                    if (toEnemy.LengthSq < 20 * 20)
                    {
                        toEnemy.Norm();
                        toEnemy = toEnemy * 30;
                        action.target_velocity_x = toEnemy.x;
                        action.target_velocity_z = toEnemy.z;
                        if (toEnemy.LengthSq < 2.5 * 2.5)
                            action.jump_speed = 15;
                        return;
                    }
                }

                for (int i = nearBallTouch; i < 200; i++)
                {
                    if (ballWays[nearBallTouch].ball.y < 3)
                    {
                        nearBallTouch = i;
                        break;
                    }
                }
                posBall = ballWays[nearBallTouch].ball.Pos();

                double dx = posBall.x;
                if (Math.Abs(me.x - posBall.x) < 4)
                {
                    if (me.x > posBall.x)
                        dx = dx + 4;
                    if (me.x < posBall.x)
                        dx = dx - 4;
                }
                double dz = posBall.z - 6;
                if (dz < -42)
                    dz = -42;
                Vector3 toBall = MoveTo(me, dx, 0, dz);
                action.target_velocity_x = toBall.x;
                action.target_velocity_y = 0;
                action.target_velocity_z = toBall.z;
                actionList = new List<myAction>();
                return;
            }







            actionList = new List<myAction>();
            if (nearBallTouch < 0)
                nearBallTouch = 0;


            if (me.z > game.ball.z && game.ball.velocity_z > 0)
            {
                for (int i = 0; i <= 190; i++)
                {
                    if (me.z < ballWays[i].ball.z)
                        break;
                    if (Math.Abs(me.x - ballWays[i].ball.x) < 3 && ballWays[i].ball.y < 3.5)
                    {
                        double dx = 0;
                        dx = -game.ball.x + 4;
                        Vector3 toBall = MoveToVec(me, dx, 0, me.z + 1);
                        action.target_velocity_x = toBall.x;
                        action.target_velocity_y = toBall.y;
                        action.target_velocity_z = toBall.z;
                        return;
                    }
                }
            }



            if (game.robots.Length == 6 && firstTick > 0 && me.z < game.ball.z && r2.z < game.ball.z && r3.z < game.ball.z && (me.z > r2.z || me.z > r3.z) &&
                (me.z < r2.z || me.z < r3.z) && game.ball.velocity_z > 0)
            {
                Vector3 toBall = MoveTo(me, game.ball.x, 0, -40);
                action.target_velocity_x = toBall.x;
                action.target_velocity_y = 0;
                action.target_velocity_z = toBall.z;
                actionList = new List<myAction>();
                return;
            }



            //----------------------------------------------------------------------------------------------------
            //----------------------------------------------------------------------------------------------------


            double bestVel = -100;
            double bestScore = firstTickGoalScore;
            if (bestScore == 0)
                bestScore = curScore;
            int error = 0;

            int cntCalC1 = 120;
            int cntCalC2 = 12;
            int cntCalC3 = 1;
            int cntCalC4 = 5;
            int cntCalC5Diff = 1;

            if (AVG > 15)
            {
                cntCalC3 = 1;
                cntCalC2 = 8;
                cntCalC1 = 60;
                cntCalC4 = 3;
                cntCalC5Diff = 1;
            }
            if (AVG < 10)
            {
                cntCalC3 = 4;
                cntCalC2 = 24;
                cntCalC1 = 190;
                cntCalC4 = 7;
                cntCalC5Diff = 1;
            }

            actionList = new List<myAction>();
            if (nearBallTouch < 0)
                nearBallTouch = 0;

            if (nearBallTouch < firstTick - game.current_tick && firstTick - game.current_tick < 50)
            {
                //nearBallTouch = firstTick - game.current_tick + 10;
            }
            int add = cntCalC3;
            int realCalc = 0;
            for (int i = nearBallTouch; i < calcCount; i++)
            {
                if (error > 50)
                    break;
                if (realCalc > 30)
                    break;
                if (i > cntCalC1)
                    break;
                if (ballWays[i].ball.y > 6 && me.nitro_amount == 0)
                    continue;
                if (ballWays[i].ball.y > 9)
                    continue;
                if (ballWays[i].ball.z > 42)
                {
                    Vector3 vec = MoveTo(me, game.ball.x, 0, game.ball.z - 6);
                    action.target_velocity_x = vec.x;
                    action.target_velocity_y = vec.y;
                    action.target_velocity_z = vec.z;
                    break;
                }
                if (ballWays[i].ball.z < -42)
                {
                    break;
                }

                if (me.z > ballWays[i].ball.z + 10)
                    continue;

                if (me.z < -38 && ballWays[i].ball.z > 0)
                    continue;


                double lenToBall = (me.x - ballWays[i].ball.x) * (me.x - ballWays[i].ball.x) +
                                   (me.z - ballWays[i].ball.z) * (me.z - ballWays[i].ball.z);
                realCalc++;
                if (lenToBall < 6 * 6)
                {
                    List<myAction> mTmp = getActionDirect(new myRobot(me), game, ballWays[i].ball.Pos(), i);
                    if (mTmp != null)
                    {
                        actionList = mTmp;
                        action.target_velocity_x = actionList[0].action.target_velocity_x;
                        action.target_velocity_y = actionList[0].action.target_velocity_y;
                        action.target_velocity_z = actionList[0].action.target_velocity_z;
                        action.jump_speed = actionList[0].action.jump_speed;
                        action.use_nitro = actionList[0].action.use_nitro;
                        return;
                    }
                }


                int xGoal = 0;
                if (me.x > 0 && me.x > ballWays[i].ball.x)
                    xGoal = -10;
                if (me.x < 0 && me.x < ballWays[i].ball.x)
                    xGoal = 10;
                posBall = new Vector3(ballWays[i].ball.x, 0, ballWays[i].ball.z);
                Vector3[] VelGoals = new Vector3[8];
                VelGoals[0] = new Vector3(xGoal, 7, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[1] = new Vector3(0, 7, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[2] = new Vector3(30, 0, (42 - posBall.z) * 0.5 - 4) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[3] = new Vector3(-30, 0, (42 - posBall.z) * 0.5 - 4) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[4] = new Vector3(0, 0, 1);
                VelGoals[5] = new Vector3(5, 7, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[6] = new Vector3(-5, 7, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, ballWays[i].ball.y, posBall.z);
                VelGoals[7] = new Vector3(0, 0, 2);
                if (ballWays[i].ball.z < -33 && Math.Abs(ballWays[i].ball.x) > 14)
                {
                    VelGoals[0] = new Vector3(0, 0, -2);
                    VelGoals[1] = new Vector3(0, 2, -2);
                    VelGoals[2] = new Vector3(2, 0, 0);
                    VelGoals[3] = new Vector3(-2, 0, 0);
                    VelGoals[4] = new Vector3(0, 0, -3);
                }
                for (int d = 0; d <= 1; d++)
                {
                    if (d == 1 && AVG > 14)
                        continue;
                    for (int v = 0; v < cntCalC4; v++)
                    {
                        if (v == 5 && ballWays[i].ball.z > 30 && Math.Abs(ballWays[i].ball.x) > 15)
                            continue;
                        for (int k = -3; k < cntCalC2; k = k + 3)
                        {
                            List<myAction> actionListTmp = new List<myAction>();
                            posBall = new Vector3(ballWays[i].ball.x, ballWays[i].ball.y, ballWays[i].ball.z);
                            vecToGoal = VelGoals[v];

                            vecToGoal.Norm();
                            double diffCalc = d;
                            if (cntCalC5Diff <= 1)
                                diffCalc = 2;
                            posBall = ballWays[i].ball.Pos() - vecToGoal * 1;
                            List<Model.Action> aList = null;
                            Vector3 myVelVec = new Vector3(vecToGoal);
                            if (d == 1)
                            {
                                myVelVec = ballWays[i].ball.Pos() - new myRobot(me).Pos();
                                myVelVec.y = 0;
                                myVelVec.Norm();
                            }

                            aList = getWayToPosFast(new myRobot(me),
                                new Vector3(posBall.x, ballWays[i].ball.y, posBall.z),
                                myVelVec, i, k, ballWays[i].ball.Pos(), 5);
                            if (aList == null)
                                continue;
                            if (aList != null)
                            {

                                myRobot rCalc = new myRobot(me);
                                myBall calcBall = new myBall(game.ball);
                                int tickNumber = game.current_tick;
                                isTouchBall = false;
                                int cntBallWay = 0;
                                foreach (Model.Action a in aList)
                                {
                                    calcBall = new myBall(ballWays[cntBallWay].ball);
                                    cntBallWay++;
                                    rCalc.action.target_velocity_x = a.target_velocity_x;
                                    rCalc.action.target_velocity_y = a.target_velocity_y;
                                    rCalc.action.target_velocity_z = a.target_velocity_z;
                                    rCalc.action.jump_speed = a.jump_speed;
                                    rCalc.action.use_nitro = a.use_nitro;
                                    ma = new myAction(rCalc.action, tickNumber)
                                    {
                                        ball = new myBall(calcBall),
                                        r = new myRobot(rCalc)
                                    };
                                    tickNumber++;
                                    actionListTmp.Add(ma);
                                    Update(ref rCalc, ref calcBall, true);
                                    double len01 = (rCalc.Pos() - calcBall.Pos()).Length;
                                    if (isTouchBall)
                                    {

                                        actionListTmp.Last().action.jump_speed = 15;
                                        actionListTmp.Last().action.use_nitro = true;
                                        actionListTmp.Last().action.target_velocity_x = 30;
                                        actionListTmp.Last().action.target_velocity_y = 30;
                                        actionListTmp.Last().action.target_velocity_z = 30;
                                        break;
                                    }
                                }

                                if (!isTouchBall)
                                {
                                    error++;
                                    continue;
                                }

                                if (rCalc.z > calcBall.z && calcBall.velocity_z < 0)
                                {
                                    error++;
                                    continue;
                                }

                                double sc = calcScore(calcBall, game);
                                /*
                                                                isTouchBall = false;
                                                                myBall rndBall =  new myBall(ma.ball);
                                                                myRobot rndR = new myRobot(ma.r);
                                                                Update(ref rndR, ref rndBall, true,0.40);
                                                                double sc1 = calcScore(rndBall, game);
                                                                rndBall = new myBall(ma.ball);
                                                                rndR = new myRobot(ma.r);
                                                                isTouchBall = false;
                                                                Update(ref rndR, ref rndBall, true, 0.50);
                                                                rndBall = new myBall(ma.ball);
                                                                double sc2 = calcScore(rndBall, game);
                                                                sc = (sc + sc1 + sc2) / 3;
                                */
                                if (bestScore == 0 || sc > bestScore)
                                {
                                    bestScore = sc;
                                    actionList = actionListTmp;
                                    action.target_velocity_x = aList[0].target_velocity_x;
                                    action.target_velocity_z = aList[0].target_velocity_z;
                                    if (bestScore > 2000)
                                        return;
                                    //Console.WriteLine("Tick="  + game.current_tick + " Cnt=" + actionList.Count + " Vector= " + v + " score=" + sc + " add=" + add + " k=" + k + " error=" + error);
                                }
                            }
                        }
                    }
                }

                if (actionList.Count > 0)
                {
                    add--;
                    if (add <= 0)
                    {
                        action.target_velocity_x = actionList[0].action.target_velocity_x;
                        action.target_velocity_z = actionList[0].action.target_velocity_z;
                        return;
                    }
                }
            }

            if (actionList.Count > 0)
            {
                action.target_velocity_x = actionList[0].action.target_velocity_x;
                action.target_velocity_z = actionList[0].action.target_velocity_z;
                return;
            }


            if (actionSave != null && actionSave.Count > 0)
            {
                actionList = actionSave;
                action.target_velocity_x = actionList[0].action.target_velocity_x;
                action.target_velocity_z = actionList[0].action.target_velocity_z;
                return;
            }





            actionList = new List<myAction>();

            int tickNumberDir = game.current_tick;
            myRobot rDirect = new myRobot(me);
            myBall bDirect = new myBall(game.ball);
            isTouchBall = false;
            for (int i = 0; i < 35; i++)
            {
                if (nearBallTouch > 35 || game.ball.velocity_z > 0)
                    break;
                if (i >= calcCount)
                    break;
                Vector3 posBallDir = new Vector3(ballWays[i].ball.x, 0, ballWays[i].ball.z);
                Vector3 velDir = posBallDir - new Vector3(rDirect.x, 0, rDirect.z);
                double lenDir = velDir.Length;
                double mySpeed = new Vector3(rDirect.velocity_x, 0, rDirect.velocity_z).Length;
                velDir.Norm();
                velDir = velDir * ROBOT_MAX_GROUND_SPEED;
                rDirect.action.target_velocity_x = velDir.x;
                rDirect.action.target_velocity_z = velDir.z;
                rDirect.action.jump_speed = 0;
                if (lenDir < 99 && mySpeed > 0)
                {
                    int index = (int)((lenDir - 2) / (mySpeed / 60));
                    if (index < 15)
                    {
                        double by = ballWays[i + 15].ball.y;
                        if (index <= getFirstTickJump(new myRobot(me), by))
                        {
                            rDirect.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                            rDirect.action.target_velocity_y = 30;
                            rDirect.action.use_nitro = true;
                        }
                    }
                }
                ma = new myAction(rDirect.action, tickNumberDir)
                {
                    ball = new myBall(bDirect),
                    r = new myRobot(rDirect)
                };
                tickNumberDir++;
                actionList.Add(ma);
                Update(ref rDirect, ref bDirect, true);
                if (isTouchBall)
                    break;
            }

            if (isTouchBall && bDirect.velocity_z > 0)
            {
                if (isGoal(bDirect) || (game.ball.velocity_z < 0))
                {
                    action.target_velocity_y = 0;
                    action.target_velocity_x = actionList[0].action.target_velocity_x;
                    action.target_velocity_z = actionList[0].action.target_velocity_z;
                    action.jump_speed = actionList[0].action.jump_speed;
                    return;
                }

            }

            actionList = new List<myAction>();

            int fastnearBallTouch = nearBallTouch;

            double diff = 2;
            if (isMneGoal != null)
                diff = 0;
            myRobot rTmp = new myRobot(me);
            int tickCnt = game.current_tick;
            //            Console.WriteLine("No move Tick=" + game.current_tick);

            if (me.z < r2.z && me.z < r3.z)
            {
                double dx = 0;
                if (game.ball.x > 15)
                    dx = 5;
                if (game.ball.x < -15)
                    dx = -5;
                Vector3 vec = MoveTo(me, dx, 0, -40);
                action.target_velocity_x = vec.x;
                action.target_velocity_y = 0;
                action.target_velocity_z = vec.z;
                return;
            }


            for (int i = nearBallTouch; i < 200; i++)
            {
                if (ballWays[nearBallTouch].ball.y < 3)
                {
                    nearBallTouch = i;
                    break;
                }
            }

            for (int i = 0; i < 15; i++)
            {
                if (i > calcCount)
                    break;
                myBall b = new myBall(ballWays[nearBallTouch].ball);
                double dx = b.x;
                double len = (new Vector3(me.x, 0, me.z) - new Vector3(r2.x, 0, r2.z)).Length;

                posBall = new Vector3(dx, 0, b.z - 2);
                if (posBall.z < -39)
                {
                    posBall.z = -39;
                }
                velToBall = posBall - new Vector3(me.x, 0, me.z);
                velToBall.Norm();
                velToBall = velToBall * ROBOT_MAX_GROUND_SPEED;
                rTmp.action.target_velocity_x = velToBall.x;
                rTmp.action.target_velocity_z = velToBall.z;
                ma = new myAction(rTmp.action, tickCnt)
                {
                    ball = new myBall(ballWays[i].ball),
                    r = new myRobot(rTmp)
                };
                tickCnt++;
                actionList.Add(ma);
                Update(ref rTmp, ref b);
            }

            action.target_velocity_x = actionList[0].action.target_velocity_x;
            action.target_velocity_z = actionList[0].action.target_velocity_z;
            return;

            int startTouch = nearBallTouch;
            int bestTouchTick = -1;
            int canTouchTick = -1;
            int TouchTickNopause = -1;
            for (int i = startTouch; i < 200; i++)
            {
                if (ballWays[i].ball.y > 4.5 + 3)
                    continue;
                myRobot checkRobot = new myRobot(me);
                posBall = new Vector3(ballWays[i].ball.x, 0, ballWays[i].ball.z);
                vecToGoal = new Vector3(0, 0, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, 0, posBall.z);
                vecToGoal.Norm();
                bestVelToBall = new Vector3(vecToGoal.x, 0, vecToGoal.z);
                vecToGoal = vecToGoal * diff;
                posBall = posBall - vecToGoal;
                velToBall = posBall - new Vector3(me.x, 0, me.z);
                velToBall.Norm();
                velToBall = velToBall * ROBOT_MAX_GROUND_SPEED;
                checkRobot.action.target_velocity_x = velToBall.x;
                checkRobot.action.target_velocity_z = velToBall.z;
                checkRobot.action.target_velocity_y = 0;
                double minDst = getLenToBallSq(checkRobot, ballWays[i].ball);
                nearBallTouch = -1;
                for (int j = 1; j <= i; j++)
                {

                    velToBall = posBall - new Vector3(checkRobot.x, 0, checkRobot.z);
                    velToBall.Norm();
                    Vector3 diffVec = bestVelToBall - velToBall;

                    if (Math.Abs(diffVec.x) > 0.4 && Math.Abs(diffVec.x) > Math.Abs(diffVec.z))
                    {
                        velToBall.z = 0;
                        if (checkRobot.x > posBall.x)
                            velToBall.z = -1;
                        else
                            velToBall.z = 1;
                    }

                    double diffVecX = bestVelToBall.x - velToBall.x;
                    velToBall = velToBall * ROBOT_MAX_GROUND_SPEED;
                    checkRobot.action.target_velocity_x = velToBall.x;
                    checkRobot.action.target_velocity_z = velToBall.z;
                    checkRobot.action.target_velocity_y = 0;
                    checkRobot.action.jump_speed = 0;
                    if (i - j <= getTickToJump(ballWays[i].ball.y) && ballWays[i].ball.y > 2.5)
                        checkRobot.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                    Sim_Robot_tick(ref checkRobot);
                    double dst = getLenToBallSq(checkRobot, ballWays[i].ball);
                    if (ballWays[i].ball.z > checkRobot.z && dst <= (checkRobot.radius + BALL_RADIUS) * (checkRobot.radius + BALL_RADIUS))
                    {
                        nearBallTouch = i;
                        if (bestTouchTick == -1 || bestTouchTick > nearBallTouch)
                            canTouchTick = j;
                        break;
                    }
                }

                double dstAfter = getLenToBallSq(checkRobot, ballWays[i].ball);
                if (TouchTickNopause == -1 && ballWays[i].ball.z > checkRobot.z && dstAfter <= (checkRobot.radius + BALL_RADIUS) * (checkRobot.radius + BALL_RADIUS))
                {
                    TouchTickNopause = i;
                }


                if (nearBallTouch == -1)
                    continue;
                if (bestTouchTick == -1 || bestTouchTick > nearBallTouch)
                {
                    bestTouchTick = nearBallTouch;
                }
            }

            nearBallTouch = bestTouchTick;

            if (bestTouchTick > canTouchTick + 5)
            {
                Vector3 ballbestTouchTick = new Vector3(ballWays[canTouchTick + 5].ball.x, 0, ballWays[canTouchTick + 5].ball.z);
                Vector3 ballbestTouchTickToGoal = ballbestTouchTick - new Vector3(0, 0, 42);
                ballbestTouchTickToGoal.Norm();
                ballbestTouchTickToGoal = ballbestTouchTickToGoal * 10;
                ballbestTouchTick = ballbestTouchTick - ballbestTouchTickToGoal;
                Vector3 toPauseVec = MoveTo(me, ballbestTouchTick.x, 0, ballbestTouchTick.z);
                action.target_velocity_x = toPauseVec.x;
                action.target_velocity_z = toPauseVec.z;
                return;
            }

            nearBallTouch = TouchTickNopause;

            if (nearBallTouch >= 100 || nearBallTouch == -1)
            {
                Vector3 toBall = MoveToVec(me, game.ball.x, 0, game.ball.z - 10);
                action.target_velocity_x = toBall.x;
                action.target_velocity_y = toBall.y;
                action.target_velocity_z = toBall.z;
                return;
            }




            double bestSpeedBall = -MAX_ENTITY_SPEED;
            for (int j = 0; j < 1; j++)
            {
                nearBallTouch = nearBallTouch;
                myBall calcBall = new myBall(game.ball);
                myRobot rCalc = new myRobot(me);
                actionList = new List<myAction>();
                posBall = new Vector3(ballWays[nearBallTouch].ball.x, 0, ballWays[nearBallTouch].ball.z);
                vecToGoal = new Vector3(0, 0, rules.arena.depth * 0.5 + 2) - new Vector3(posBall.x, 0, posBall.z);
                vecToGoal.Norm();
                bestVelToBall = new Vector3(vecToGoal.x, 0, vecToGoal.z);
                vecToGoal = vecToGoal * diff;
                posBall = posBall - vecToGoal;
                velToBall = posBall - new Vector3(me.x, 0, me.z);
                velToBall.Norm();
                velToBall = velToBall * ROBOT_MAX_GROUND_SPEED;
                isTouchBall = false;


                int tickNumber = game.current_tick;

                for (int i = 0; i <= nearBallTouch + 1; i++)
                {
                    velToBall = posBall - new Vector3(rCalc.x, 0, rCalc.z);
                    velToBall.Norm();

                    Vector3 diffVec = bestVelToBall - velToBall;
                    velToBall = diffVec;


                    if (Math.Abs(diffVec.x) > 0.4 && Math.Abs(diffVec.x) > Math.Abs(diffVec.z))
                    {
                        velToBall.z = 0;
                        if (rCalc.x > posBall.x)
                            velToBall.z = -1;
                        else
                            velToBall.z = 1;
                    }
                    velToBall = velToBall * ROBOT_MAX_GROUND_SPEED;
                    rCalc.action.target_velocity_x = velToBall.x;
                    rCalc.action.target_velocity_z = velToBall.z;
                    rCalc.action.jump_speed = 0;

                    if (nearBallTouch - i <= 15)
                    {
                        if (nearBallTouch - i - j <= getTickToJump(ballWays[nearBallTouch].ball.y))
                            rCalc.action.jump_speed = ROBOT_MAX_JUMP_SPEED;
                    }
                    ma = new myAction(rCalc.action, tickNumber)
                    {
                        ball = new myBall(calcBall),
                        r = new myRobot(rCalc)
                    };
                    tickNumber++;
                    actionList.Add(ma);
                    double testLen = getLenToBallSq(rCalc, calcBall);
                    double testZ = calcBall.z - ballWays[i].ball.z;
                    if (i > nearBallTouch - 3)
                        nearBallTouch = nearBallTouch;
                    Update(ref rCalc, ref calcBall);
                    if (isTouchBall)
                        break;
                }

                if (isTouchBall == false && j == 0)
                {
                    action.target_velocity_y = 0;
                }

                myBall bTest = new myBall(calcBall);
                bTest = calcBall;
                if (isTouchBall)
                {
                    if (calcBall.velocity_z > bestSpeedBall)
                    {
                        bestSpeedBall = calcBall.velocity_z;
                        action.target_velocity_x = actionList[0].action.target_velocity_x;
                        action.target_velocity_z = actionList[0].action.target_velocity_z;

                    }
                    if (calcBall.velocity_z > 0 && isGoal(calcBall))
                    {
                        if (j != 0)
                            action.target_velocity_y = 0;
                        action.target_velocity_x = actionList[0].action.target_velocity_x;
                        action.target_velocity_z = actionList[0].action.target_velocity_z;
                        action.jump_speed = actionList[0].action.jump_speed;
                        return;
                    }
                }
            }
            //            action.target_velocity_x = actionList[0].action.target_velocity_x;
            //            action.target_velocity_z = actionList[0].action.target_velocity_z;
            if (bestSpeedBall > 1)
                return;
            actionList = null;

            Vector3 lastMove = MoveTo(me, game.ball.x, 0, game.ball.z - 8);
            action.target_velocity_x = lastMove.x;
            action.target_velocity_y = lastMove.y;
            action.target_velocity_z = lastMove.z;
            return;
        }

        public double getNearEnemy(myBall b, Game game, int tick)
        {
            double len = 80 * 80 * 80;
            if (b.y > 8)
                return len;
            myRobot rNear = null;
            foreach (Robot r in game.robots)
            {
                if (r.is_teammate == false)
                    continue;
                if (r.z < b.z)
                    continue;
                double dx, dy, dz;
                if (tick > 15)
                    tick = 0;
                dx = r.x + r.velocity_x / 60 * tick;
                dy = r.y + r.velocity_y / 60 * tick;
                dz = r.z + r.velocity_z / 60 * tick;

                double lenSq = (b.x - dx) * (b.x - dx) + (b.y - dy) * (b.y - dy) + (b.z - dz) * (b.z - dz);
                if (len == 0 || len > lenSq)
                {
                    rNear = new myRobot(r);
                    len = lenSq;
                }
            }
            return len;

        }



        public double calcScore(myBall ball, Game game)
        {
            if (ball.velocity_z < 0)
                return ball.velocity_z;
            double score = Math.Abs(ball.velocity_y) + ball.velocity_z;
            double len = 0;
            myBall cBall = new myBall(ball);
            bool isNear = false;
            for (int i = 0; i < 200; i++)
            {

                if (cBall.velocity_z < 0)
                    break;

                double lenNear = getNearEnemy(cBall, game, i);
                if (lenNear < 5 * 5 && cBall.z < 0)
                    score = score - 1000;
                if (lenNear < 7 * 7 && cBall.z < 0)
                    score = score - 50;

                if (cBall.z > Myrules.arena.depth * 0.5 + BALL_RADIUS)
                {
                    score = score + 500;
                    if (keepEnemy != null)
                    {
                        int jump = getFirstTickJump(keepEnemy, cBall.y);
                        Vector3 toB = keepEnemy.Pos() - cBall.Pos();
                        toB.y = 0;
                        double tickToTouch = (toB.Length - 3) * 2;
                        if (tickToTouch + jump > i)
                            score = score + 2000;
                    }

                    break;
                }
                Sim_ball(ref cBall);
            }

            return score;
        }
        public bool isGoal(myBall ball)
        {
            myBall b = new myBall(ball);
            for (int i = 0; i < 200; i++)
            {
                if (b.z > Myrules.arena.depth * 0.5 + BALL_RADIUS)
                    return true;
                if (b.velocity_z <= 0)
                    return false;
                Sim_ball(ref b);
            }
            return false;
        }
        public bool isGoalMne(myBall b)
        {
            for (int i = 0; i < 300; i++)
            {
                if (b.z < -Myrules.arena.depth * 0.5 - BALL_RADIUS)
                    return true;
                if (b.velocity_z >= 0)
                    return false;
                Sim_ball(ref b);
            }
            return false;
        }

        public void Update(ref myRobot robot, ref myBall ball, bool isFull = false, double RND = 0.45)
        {
            myRobot sourceR = new myRobot(robot);
            myBall sourceBall = new myBall(ball);

            Sim_Robot_tick(ref robot, isFull);
            Sim_ball(ref ball);
            if (CheckColliseBallRobot(robot, ball))
            {
                robot = sourceR;
                ball = sourceBall;
                double delta_time = 1 / TICKS_PER_SECOND;
                double delta_time_tick = delta_time / (double)MICROTICKS_PER_TICK;
                for (int i = 0; i <= (int)MICROTICKS_PER_TICK - 1; i++)
                {
                    {
                        Vector3 robot_action_target_velocity = new Vector3(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z);
                        Vector3 robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                        if (robot.touch)
                        {
                            Vector3 target_velocity = new Vector3(robot_action_target_velocity);
                            target_velocity.Clamp(ROBOT_MAX_GROUND_SPEED);
                            target_velocity -= new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) * dot(new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z), target_velocity);
                            Vector3 target_velocity_change = target_velocity - new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                            if (target_velocity_change.Length > 0)
                            {
                                double acceleration = ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal_y);
                                Vector3 target_velocity_change_tmp = new Vector3(target_velocity_change);
                                target_velocity_change_tmp.Norm();
                                target_velocity_change_tmp = target_velocity_change_tmp * acceleration * delta_time_tick;
                                target_velocity_change_tmp.Clamp(target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + target_velocity_change_tmp.x;
                                robot.velocity_y = robot.velocity_y + target_velocity_change_tmp.y;
                                robot.velocity_z = robot.velocity_z + target_velocity_change_tmp.z;
                            }
                        }
                        if (robot.action.use_nitro)
                        {
                            Vector3 target_velocity_change = new Vector3(robot_action_target_velocity - robot_velocity);
                            target_velocity_change.Clamp(robot.nitro_amout * NITRO_POINT_VELOCITY_CHANGE);
                            if (target_velocity_change.Length > 0)
                            {
                                Vector3 target_velocity_change_norm = new Vector3(target_velocity_change);
                                target_velocity_change_norm.Norm();
                                Vector3 acceleration = target_velocity_change_norm * ROBOT_NITRO_ACCELERATION;
                                Vector3 velocity_change = Clamp(acceleration * delta_time_tick, target_velocity_change.Length);
                                robot.velocity_x = robot.velocity_x + velocity_change.x;
                                robot.velocity_y = robot.velocity_y + velocity_change.y;
                                robot.velocity_z = robot.velocity_z + velocity_change.z;
                                robot.nitro_amout -= velocity_change.Length / NITRO_POINT_VELOCITY_CHANGE;
                            }
                        }
                        robot.Move(delta_time_tick);
                        robot.radius = ROBOT_MIN_RADIUS + (Myrules.ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
                        robot.radius_change_speed = robot.action.jump_speed;

                        ball.Move(delta_time_tick);
                        ball.colliseArena();
                        Entity a = new Entity(robot);
                        Entity b = new Entity(ball);
                        collide_entities(a, b, RND);
                        robot.x = a.position.x; robot.y = a.position.y; robot.z = a.position.z;
                        robot.velocity_x = a.velocity.x; robot.velocity_y = a.velocity.y; robot.velocity_z = a.velocity.z;
                        ball.x = b.position.x; ball.y = b.position.y; ball.z = b.position.z;
                        ball.velocity_x = b.velocity.x;
                        ball.velocity_y = b.velocity.y;
                        ball.velocity_z = b.velocity.z;
                        robot.colliseArena();
                    }

                }
            }
        }

        public bool CheckColliseBallRobot(myRobot r, myBall b)
        {
            double len = (r.x - b.x) * (r.x - b.x) + (r.y - b.y) * (r.y - b.y) + (r.z - b.z) * (r.z - b.z);
            double len1 = (r.radius + BALL_RADIUS) * (r.radius + BALL_RADIUS);
            if (len < len1)
                return true;
            return false;
        }

        public void CalcBallWays(Game game)
        {
            double eps = 1E-6;
            BallWay bway = null;
            BallWay last = null;
            myBall b = null;
            myBall bTest = null;
            int nearID = 0;
            int nearTouch = -1;
            if (ballWays != null)
            {
                bway = ballWays.FirstOrDefault(w => w.tick == game.current_tick);
                last = ballWays.LastOrDefault();
            }

            calcCount = 200;

            myRobot nearR = null;
            double dst = 0;
            keepEnemy = null;
            foreach (var r in game.robots)
            {
                if (!r.is_teammate)
                    if (keepEnemy == null || r.z > keepEnemy.z)
                        keepEnemy = new myRobot(r);
                if (r.is_teammate)
                    continue;
                if (r.touch == false && r.y < 1.5 && r.velocity_y > 0)
                {
                    Vector3 v = new Vector3(game.ball.x - r.x, game.ball.y - r.y, game.ball.z - r.z);
                    if (dst == 0 || v.LengthSq < dst)
                    {
                        dst = v.LengthSq;
                        nearR = new myRobot(r);

                        nearR.action.target_velocity_x = nearR.velocity_x;
                        nearR.action.target_velocity_z = nearR.velocity_z;
                        if (nearR.nitro_amout > 0)
                        {
                            nearR.action.use_nitro = true;
                            nearR.action.target_velocity_y = 30;
                        }
                        nearID = r.id;
                    }
                }
            }

            if (keepEnemy != null)
            {
                if (keepEnemy.x > 0)
                    goalX = -7;
                if (keepEnemy.x < 0)
                    goalX = 7;
            }
            //nearID = 0;

            if (nearID == 0 && false)
            {
                for (int i = 0; i < 30; i++)
                {
                    myBall cBall = new myBall(game.ball);
                    foreach (var r in game.robots)
                    {
                        if (r.is_teammate || r.z < cBall.z || r.touch == false)
                            continue;
                        myRobot mr = new myRobot(r);
                        mr.x = mr.x + mr.velocity_x / 60 * i;
                        mr.z = mr.z + mr.velocity_z / 60 * i;
                        double len = (cBall.x - mr.x) * (cBall.x - mr.x) + (cBall.z - mr.z) * (cBall.z - mr.z);
                        if (len < 2 * 2)
                        {
                            nearTouch = i;
                            nearID = r.id;
                            break;
                        }
                    }
                    if (nearTouch >= 0)
                        break;
                    Sim_ball(ref cBall);
                }
            }

            if (bway != null)
            {
                b = bway.ball;
                if (nearR != null)
                    if (bway.nearRid != nearID)
                    {
                        //Console.WriteLine("Recalc robot tick=" + game.current_tick.ToString());
                        bway = null;
                    }

                if ((Math.Abs(b.x - game.ball.x) > eps) || (Math.Abs(b.y - game.ball.y) > eps) || (Math.Abs(b.z - game.ball.z) > eps) || (Math.Abs(b.velocity_x - game.ball.velocity_x) > eps) || (Math.Abs(b.velocity_y - game.ball.velocity_y) > eps) || (Math.Abs(b.velocity_z - game.ball.velocity_z) > eps))
                {
                    //                    Console.WriteLine("Recalc tick=" + game.current_tick.ToString());
                    bway = null;
                }
            }

            isTouchBall = false;
            if (bway == null || last == null || nearTouch > 0)
            {
                actionList = new List<myAction>();
                actionList1 = new List<myAction>();
                actionList2 = new List<myAction>();
                actionList3 = new List<myAction>();
                ballWays = new List<BallWay>();
                firstTick = 0;
                firstTickGoalScore = 0;
                b = new myBall(game.ball);
                bTest = new myBall(game.ball);
                for (int i = game.current_tick; i < game.current_tick + 200; i++)
                {

                    BallWay bw = new BallWay(b, i);
                    bw.nearRid = nearID;
                    ballWays.Add(bw);
                    if (Math.Abs(b.z) > 43)
                        continue;

                    //                    bTest = new myBall(b);
                    //                    Sim_ball_full(b);
                    if (nearR != null && nearR.touch == false && isTouchBall == false)
                    {
                        double len = (nearR.Pos() - b.Pos()).Length;
                        if (len < 4)
                            nearR.action.jump_speed = 15;
                        Update(ref nearR, ref b, true);
                        if (nearR.touch)
                            nearID = 0;
                        //if (isTouchBall)
                        //Console.WriteLine("Touch tick=" +i);
                    }
                    else
                        Sim_ball(ref b);
                    //                    Console.WriteLine(bTest.y - b.y);
                    //                    Sim_ball(bTest);
                    if (nearTouch == i - game.current_tick)
                    {
                        b.velocity_x = -b.velocity_x;
                        b.velocity_y = -b.velocity_y;
                        b.velocity_z = -b.velocity_z;
                        nearID = 0;
                    }
                }
                return;
            }

            ballWays.RemoveAll(w => w.tick < game.current_tick);
            b = last.ball;
            int tick = last.tick;
            while (ballWays.Count < 200)
            {
                tick++;
                Sim_ball(ref b);
                BallWay bw = new BallWay(b, tick);
                ballWays.Add(bw);
            }
            if (actionList == null)
                actionList = new List<myAction>();
            if (actionList1 == null)
                actionList1 = new List<myAction>();
            if (actionList2 == null)
                actionList2 = new List<myAction>();
            if (actionList3 == null)
                actionList3 = new List<myAction>();
            firstTick = 0;
            myRobot rFirst = null;
            myAction aFirst = null;
            int numFirstList = 0;
            if (actionList1.Count > 0)
            {
                myBall testBall = new myBall(actionList1.Last().ball);
                myRobot rTest = new myRobot(actionList1.Last().r);
                isTouchBall = false;
                Update(ref rTest, ref testBall);
                if (isTouchBall)
                {
                    firstTick = actionList1.Last().tick;
                    aFirst = actionList1.Last();
                    numFirstList = 1;
                }
            }
            if (actionList2.Count > 0)
                if (actionList2.Last().tick < firstTick || firstTick == 0)
                {
                    myBall testBall = new myBall(actionList2.Last().ball);
                    myRobot rTest = new myRobot(actionList2.Last().r);
                    isTouchBall = false;
                    Update(ref rTest, ref testBall);
                    if (isTouchBall)
                    {
                        firstTick = actionList2.Last().tick;
                        aFirst = actionList2.Last();
                        numFirstList = 2;
                    }
                }
            if (actionList3.Count > 0)
                if (actionList3.Last().tick < firstTick || firstTick == 0)
                {
                    myBall testBall = new myBall(actionList3.Last().ball);
                    myRobot rTest = new myRobot(actionList3.Last().r);
                    isTouchBall = false;
                    Update(ref rTest, ref testBall);
                    if (isTouchBall)
                    {
                        firstTick = actionList3.Last().tick;
                        aFirst = actionList3.Last();
                        numFirstList = 3;
                    }
                }

            if (firstTick > 0 && CalcfirstTick != firstTick)
            {
                isTouchBall = false;
                myBall bFirst = new myBall(ballWays[firstTick - game.current_tick].ball);
                rFirst = new myRobot(aFirst.r);
                Update(ref rFirst, ref bFirst);
                firstTickGoalScore = calcScore(new myBall(bFirst), game);
                for (int i = firstTick - game.current_tick + 1; i < 200; i++)
                {
                    ballWays[i].ball = new myBall(bFirst);
                    Sim_ball(ref bFirst);
                }
                CalcfirstTick = firstTick;
                if (numFirstList == 1)
                {
                    actionList2 = new List<myAction>();
                    actionList3 = new List<myAction>();
                }
                if (numFirstList == 2)
                {
                    actionList1 = new List<myAction>();
                    actionList3 = new List<myAction>();
                }
                if (numFirstList == 3)
                {
                    actionList1 = new List<myAction>();
                    actionList2 = new List<myAction>();
                }
            }


        }

        public double GetNumTickToPoint(myRobot R, Vector3 vec)
        {
            vec = new Vector3(vec.x - R.x, 0, vec.z - R.z);
            vec.Norm();
            vec = vec * 1.666666666;
            Vector3 curSpeed = new Vector3(R.velocity_x, 0, R.velocity_z);
            Vector3 diff = curSpeed - vec;

            return diff.Length / 1.666666666666666666666;
        }

        public void Sim_Robot_tick(ref myRobot robot, bool isFull = false)
        {
            if (robot.action.jump_speed > 0 && robot.radius == ROBOT_RADIUS)
            {
                Sim_Robot(ref robot);
                return;
            }

            myRobot rSim = new myRobot(robot);

            Vector3 target_velocity_change_tmp = new Vector3(0, 0, 0);
            Vector3 robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
            Vector3 target_velocity_change_nitro = new Vector3(0, 0, 0);
            Vector3 target_velocity_change;
            Vector3 robot_action_target_velocity = new Vector3(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z);
            Vector3 target_velocity = new Vector3(robot_action_target_velocity);
            double delta_time = 1 / TICKS_PER_SECOND;

            if (robot.touch)
            {
                target_velocity.Clamp(ROBOT_MAX_GROUND_SPEED);
                target_velocity -= new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) * dot(new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z), target_velocity);
                target_velocity_change = target_velocity - new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                if (target_velocity_change.Length > 0)
                {
                    double acceleration = ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal_y);
                    target_velocity_change_tmp = new Vector3(target_velocity_change);
                    target_velocity_change_tmp.Norm();
                    target_velocity_change_tmp = target_velocity_change_tmp * acceleration * delta_time;
                    target_velocity_change_tmp.Clamp(target_velocity_change.Length);
                    robot.velocity_x = robot.velocity_x + target_velocity_change_tmp.x;
                    robot.velocity_y = robot.velocity_y + target_velocity_change_tmp.y;
                    robot.velocity_z = robot.velocity_z + target_velocity_change_tmp.z;
                }
            }
            if (robot.action.use_nitro)
            {
                robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                target_velocity_change = new Vector3(target_velocity.x - robot_velocity.x, target_velocity.y - robot_velocity.y, target_velocity.z - robot_velocity.z);
                target_velocity_change.Clamp(robot.nitro_amout * NITRO_POINT_VELOCITY_CHANGE);
                if (target_velocity_change.Length > 0)
                {
                    if (target_velocity_change.Length < 0.5)
                    {
                        //                        robot = rSim;
                        //                        Sim_Robot(ref robot);
                        //                        return;
                    }
                    Vector3 target_velocity_change_norm = new Vector3(target_velocity_change);
                    target_velocity_change_norm.Norm();
                    Vector3 acceleration = target_velocity_change_norm * ROBOT_NITRO_ACCELERATION;
                    target_velocity_change_nitro = Clamp(acceleration * delta_time, target_velocity_change.Length);
                    robot.velocity_x = robot.velocity_x + target_velocity_change_nitro.x;
                    robot.velocity_y = robot.velocity_y + target_velocity_change_nitro.y;
                    robot.velocity_z = robot.velocity_z + target_velocity_change_nitro.z;
                    robot.nitro_amout -= target_velocity_change_nitro.Length / NITRO_POINT_VELOCITY_CHANGE;
                }
            }
            robot.MoveTick(target_velocity_change_tmp, target_velocity_change_nitro);
            robot.radius = ROBOT_MIN_RADIUS + (Myrules.ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
            robot.radius_change_speed = robot.action.jump_speed;
            robot.colliseArena();
            if (isFull && AVG < 15 && robot.touch && Math.Abs(1 - robot.touch_normal_y) > 0.2 && robot.isMy)
            {
                robot = rSim;
                Sim_Robot(ref robot);
                return;
            }

        }

        public void Sim_Robot(ref myRobot robot)
        {
            double delta_time = 1 / TICKS_PER_SECOND;
            double delta_time_tick = delta_time / (double)MICROTICKS_PER_TICK;
            for (int i = 0; i <= (int)MICROTICKS_PER_TICK - 1; i++)
            {
                Vector3 robot_action_target_velocity = new Vector3(robot.action.target_velocity_x, robot.action.target_velocity_y, robot.action.target_velocity_z);
                Vector3 robot_velocity = new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                if (robot.touch)
                {
                    Vector3 target_velocity = new Vector3(robot_action_target_velocity);
                    target_velocity.Clamp(ROBOT_MAX_GROUND_SPEED);
                    target_velocity -= new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z) * dot(new Vector3(robot.touch_normal_x, robot.touch_normal_y, robot.touch_normal_z), target_velocity);
                    Vector3 target_velocity_change = target_velocity - new Vector3(robot.velocity_x, robot.velocity_y, robot.velocity_z);
                    if (target_velocity_change.Length > 0)
                    {
                        double acceleration = ROBOT_ACCELERATION * Math.Max(0, robot.touch_normal_y);
                        Vector3 target_velocity_change_tmp = new Vector3(target_velocity_change);
                        target_velocity_change_tmp.Norm();
                        target_velocity_change_tmp = target_velocity_change_tmp * acceleration * delta_time_tick;
                        target_velocity_change_tmp.Clamp(target_velocity_change.Length);
                        robot.velocity_x = robot.velocity_x + target_velocity_change_tmp.x;
                        robot.velocity_y = robot.velocity_y + target_velocity_change_tmp.y;
                        robot.velocity_z = robot.velocity_z + target_velocity_change_tmp.z;
                    }
                }
                if (robot.action.use_nitro)
                {
                    Vector3 target_velocity_change = new Vector3(robot_action_target_velocity - robot_velocity);
                    target_velocity_change.Clamp(robot.nitro_amout * NITRO_POINT_VELOCITY_CHANGE);
                    if (target_velocity_change.Length > 0)
                    {
                        Vector3 target_velocity_change_norm = new Vector3(target_velocity_change);
                        target_velocity_change_norm.Norm();
                        Vector3 acceleration = target_velocity_change_norm * ROBOT_NITRO_ACCELERATION;
                        Vector3 velocity_change = Clamp(acceleration * delta_time_tick, target_velocity_change.Length);
                        robot.velocity_x = robot.velocity_x + velocity_change.x;
                        robot.velocity_y = robot.velocity_y + velocity_change.y;
                        robot.velocity_z = robot.velocity_z + velocity_change.z;
                        robot.nitro_amout -= velocity_change.Length / NITRO_POINT_VELOCITY_CHANGE;
                    }
                }
                robot.Move(delta_time_tick);
                robot.radius = ROBOT_MIN_RADIUS + (Myrules.ROBOT_MAX_RADIUS - ROBOT_MIN_RADIUS) * robot.action.jump_speed / ROBOT_MAX_JUMP_SPEED;
                robot.radius_change_speed = robot.action.jump_speed;
                robot.colliseArena();
            }
        }

        public double getLen(double speed)
        {
            double res = 0;
            if (speed >= ROBOT_MAX_GROUND_SPEED)
                return speed / 60;
            res = 1.40277777777777777 + speed / 60;
            return res * 0.01;
        }
        public Vector3 MoveTo(Robot R, double x, double y, double z)
        {
            double speed = 30;
            Vector3 delta_pos = new Vector3(x, y, z) - new Vector3(R.x, R.y, R.z);
            if (delta_pos.LengthSq < 1)
            {
                return new Vector3(0, 0, 0);
            }
            if (delta_pos.LengthSq < 3)
            {
                speed = 15;
            }
            delta_pos.Norm();
            delta_pos = delta_pos * speed;
            return delta_pos;
        }

        public Vector3 MoveTo(myRobot R, double x, double y, double z)
        {
            Vector3 delta_pos = new Vector3(x, y, z) - new Vector3(R.x, R.y, R.z);
            if (delta_pos.LengthSq < 4)
            {
                return new Vector3(0, 0, 0);
            }

            delta_pos.Norm();
            delta_pos = delta_pos * ROBOT_MAX_GROUND_SPEED;
            return delta_pos;
        }

        public Vector3 MoveToTrunc(myRobot R, double x, double y, double z)
        {
            Vector3 delta_pos = new Vector3(x, y, z) - new Vector3(R.x, R.y, R.z);
            if (delta_pos.Length < (new Vector(R.velocity_x, R.velocity_z).Length / 60))
            {
                return new Vector3(0, 0, 0);
            }

            delta_pos.Norm();
            delta_pos = delta_pos * ROBOT_MAX_GROUND_SPEED;
            return delta_pos;
        }

        public Vector3 MoveToVec(Robot R, double x, double y, double z)
        {
            Vector3 delta_pos = new Vector3(x, y, z) - new Vector3(R.x, R.y, R.z);
            delta_pos.Norm();
            delta_pos = delta_pos * ROBOT_MAX_GROUND_SPEED;
            return delta_pos;
        }
        public Vector3 MoveToVec(myRobot R, double x, double y, double z)
        {
            Vector3 delta_pos = new Vector3(x, y, z) - new Vector3(R.x, 0, R.z);
            delta_pos.Norm();
            delta_pos = delta_pos * ROBOT_MAX_GROUND_SPEED;
            return delta_pos;
        }

        public double getLenToBall(myRobot r, myBall b)
        {
            return new Vector3(b.x - r.x, b.y - r.y, b.z - r.z).Length;
        }
        public double getLenToBallSq(myRobot r, myBall b)
        {
            return (r.x - b.x) * (r.x - b.x) + (r.z - b.z) * (r.z - b.z) + (r.y - b.y) * (r.y - b.y);
        }

        public int getTickToJump(myRobot me, double y)
        {
            myRobot rCalc = new myRobot(me);
            rCalc.radius_change_speed = 15;
            for (int i = 0; i < 20; i++)
            {
                if (Math.Abs(y - me.y) < 3)
                    return i;
                Sim_Robot_tick(ref rCalc);
            }
            return 0;
        }

        public int getFirstTickToJump(double y)
        {
            y = y - 2;
            if (y < 1)
                return 1;
            if (y < 1.53)
                return 2;
            if (y < 1.76)
                return 3;
            if (y < 1.98)
                return 4;
            if (y < 2.19)
                return 5;
            if (y < 2.38)
                return 6;
            if (y < 2.58)
                return 7;
            if (y < 2.78)
                return 8;
            if (y < 2.95)
                return 9;
            if (y < 3.12)
                return 10;
            if (y < 3.29)
                return 11;
            if (y < 3.445)
                return 12;
            if (y < 3.59)
                return 13;
            if (y < 3.72)
                return 14;
            if (y < 3.857)
                return 15;
            if (y < 3.977)
                return 16;
            if (y < 4.09)
                return 17;
            if (y < 4.19)
                return 18;
            if (y < 4.29)
                return 19;
            if (y < 4.37)
                return 20;
            if (y < 4.45)
                return 21;
            if (y < 4.52)
                return 22;
            if (y < 4.58888)
                return 23;
            if (y < 4.64)
                return 24;
            if (y < 4.68)
                return 25;
            if (y < 4.72)
                return 26;
            if (y < 4.75)
                return 27;
            if (y < 4.77)
                return 28;
            if (y < 4.80)
                return 30;
            return 0;
        }

        public int getOptimalTickJump(myRobot me, double y)
        {
            myRobot rCalc = new myRobot(me);
            double dst = Math.Abs(y - me.y);
            int res = 0;
            for (int i = 1; i <= 60; i++)
            {
                if (y > rCalc.y && Math.Abs(y - rCalc.y) + ESP < dst || i == 1)
                {
                    res = i;
                    dst = Math.Abs(y - rCalc.y);
                }
                else
                {
                    break;
                }
                rCalc.action.target_velocity_y = 30;
                rCalc.action.target_velocity_x = 0;
                rCalc.action.target_velocity_z = 0;
                rCalc.action.jump_speed = 15;
                rCalc.action.use_nitro = true;
                Sim_Robot_tick(ref rCalc);
            }
            if (dst < 3)
                return res;
            return 0;
        }

        public int getFirstTickJump(myRobot me, double y)
        {
            myRobot rCalc = new myRobot(me);
            double dst = Math.Abs(y - me.y);
            int res = 0;
            for (int i = 1; i <= 60; i++)
            {
                if (Math.Abs(y - rCalc.y) + ESP < dst || i == 1)
                {
                    res = i;
                    dst = Math.Abs(y - rCalc.y);
                    if (dst < 2)
                        return i;
                }
                else
                {
                    break;
                }

                rCalc.action.target_velocity_y = 30;
                rCalc.action.target_velocity_x = 0;
                rCalc.action.target_velocity_z = 0;
                rCalc.action.jump_speed = 15;
                rCalc.action.use_nitro = true;
                Sim_Robot_tick(ref rCalc);
            }
            if (dst < 3)
                return res;
            return 0;
        }

        public int getTickToJump(double y)
        {
            if (y < 2.1)
                return 1;
            if (y < 3)
                return 5;
            if (y < 3.6)
                return 8;
            if (y < 3.78)
                return 9;
            if (y < 3.958)
                return 10;
            if (y < 4.13)
                return 11;
            if (y < 4.29)
                return 12;
            if (y < 4.445)
                return 13;
            if (y < 4.73)
                return 14;
            if (y < 3.857)
                return 15;
            if (y < 3.977)
                return 16;
            if (y < 4.09)
                return 17;
            if (y < 4.19)
                return 18;
            if (y < 4.29)
                return 19;
            if (y < 4.37)
                return 20;
            if (y < 4.45)
                return 21;
            if (y < 4.52)
                return 22;
            if (y < 4.58888)
                return 23;
            if (y < 4.64)
                return 24;
            if (y < 4.68)
                return 25;
            if (y < 4.72)
                return 26;
            if (y < 4.75)
                return 27;
            if (y < 4.77)
                return 28;
            if (y < 4.80)
                return 30;
            return 60;
        }

        public int getTickToJumpNitro(double y)
        {
            if (y < 2.9)
                return 1;
            double h = 1.29726625;
            for (int i = 1; i < 50; i++)
            {
                if (h >= y)
                    return i - 1;
                h = h + 0.24976388888888;
            }

            return 60;
        }

        public bool isStartGame(Game game)
        {
            if (game.ball.x == 0 && game.ball.z == 0 && game.ball.velocity_x == 0)
                return true;
            return false;
        }

        public Model.Action getStartWay(Robot me, Game game)
        {
            Model.Action a = new Model.Action();
            double mySpeedSq = me.velocity_x * me.velocity_x + me.velocity_z * me.velocity_z;
            if (mySpeedSq < 10 * 10)
            {
                a.use_nitro = true;
                a.target_velocity_x = 0.8 * 30;
                if (me.x > 0)
                    a.target_velocity_x = -0.8 * 30;
                a.target_velocity_z = 0.2 * 30;
                return a;
            }
            Vector3 toBallVec = new Vector3(-me.x, 0, -me.z);
            int indexToTouch = (int)((toBallVec.Length - 3) / (Math.Sqrt(mySpeedSq) / 60)) - 1;
            int jump = 1;
            if (indexToTouch <= 15)
                jump = getTickToJump(ballWays[indexToTouch].ball.y);
            if (indexToTouch <= jump || indexToTouch < 4)
                a.jump_speed = 15;
            toBallVec.Norm();
            toBallVec = toBallVec * 30;
            a.target_velocity_x = toBallVec.x;
            a.target_velocity_z = toBallVec.z;
            return a;
        }

        public void ClearActionList(int id)
        {
            if (actionList == null)
                return;
            if (actionList.Count() == 0)
                return;
            actionList.RemoveAll(a => a.r.id == id);
        }

        public void initVec()
        {
            calcMoves = new List<calcMove>();

            calcMoves.Add(new calcMove(0, 0, 1, -43.5, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(1, 0, 1, -43.5140277777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(2, 0, 1, -43.5558333333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(3, 0, 1, -43.6254166666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(4, 0, 1, -43.7227777777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(5, 0, 1, -43.8479166666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(6, 0, 1, -44.0008333333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(7, 0, 1, -44.1815277777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(8, 0, 1, -44.39, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(9, 0, 1, -44.62625, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(10, 0, 1, -44.8902777777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(11, 0, 1, -45.1820833333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(12, 0, 1, -45.5016666666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(13, 0, 1, -45.8479616160328, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(14, 0, 1, -46.201515006626, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(15, 0, 1, -46.5550683972192, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(16, 0, 1, -46.9086217878125, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(17, 0, 1.01814601856092, -47.2688025227822, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(18, 0, 1.10659273636713, -47.644211870445, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(19, 0, 1.27723679066161, -48.0159167901704, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(20, 0, 1.53311690819816, -48.3595050551528, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(21, 0, 1.8682455848318, -48.6489790610397, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(22, 0, 2.26693206549991, -48.860809340961, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(23, 0, 2.69870143796323, -48.9771745437656, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(24, 0, 3.13320938795712, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(25, 0, 3.56018116962285, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(26, 0, 3.97881961795525, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(27, 0, 4.38912473295432, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(28, 0, 4.79109651462006, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(29, 0, 5.18473496295246, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(30, 0, 5.57004007795154, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(31, 0, 5.94701185961728, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(32, 0, 6.31565030794969, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(33, 0, 6.67595542294877, -49, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(34, 0, 7.02792612163773, -48.9998050234286, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(35, 0, 7.36941638529873, -48.9655868167733, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(36, 0, 7.69232411182448, -48.8763494674997, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(37, 0, 7.98899598690327, -48.7383575402918, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(38, 0, 8.25354363027483, -48.5584057132202, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(39, 0, 8.48164101646256, -48.3434060809509, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(40, 0, 8.67025942394715, -48.1001061115709, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(41, 0, 8.81738619066644, -47.8349295982147, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(42, 0, 8.92176206094211, -47.5539229017865, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(43, 0, 8.98266094410363, -47.2627842855387, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(44, 0, 8.99994958333333, -46.9669519128607, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(45, 0, 8.99486624999992, -46.6705121556727, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(46, 0, 8.98144958333318, -46.3740723984847, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(47, 0, 8.95969958333311, -46.0776326412967, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(48, 0, 8.9296162499997, -45.7811928841087, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(49, 0, 8.89119958333296, -45.4847531269208, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(50, 0, 8.84444958333289, -45.1883133697328, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(51, 0, 8.78936624999949, -44.8918736125448, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(52, 0, 8.72594958333275, -44.5954338553568, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(53, 0, 8.65419958333268, -44.2989940981688, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(54, 0, 8.57411624999927, -44.0025543409808, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(55, 0, 8.48569958333253, -43.7061145837929, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(56, 0, 8.38894958333247, -43.4096748266049, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(57, 0, 8.28386624999906, -43.1132350694169, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(58, 0, 8.17044958333232, -42.8167953122289, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(59, 0, 8.04869958333225, -42.5203555550409, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(60, 0, 7.91861624999891, -42.2239157978529, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(61, 0, 7.78019958333226, -41.927476040665, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(62, 0, 7.63344958333228, -41.631036283477, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(63, 0, 7.47836624999896, -41.334596526289, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(64, 0, 7.31494958333231, -41.038156769101, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(65, 0, 7.14319958333232, -40.741717011913, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(66, 0, 6.963116249999, -40.445277254725, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(67, 0, 6.77469958333234, -40.1488374975371, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(68, 0, 6.57794958333235, -39.8523977403491, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(69, 0, 6.37286624999902, -39.5559579831611, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(70, 0, 6.15944958333236, -39.2595182259731, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(71, 0, 5.93769958333237, -38.9630784687851, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(72, 0, 5.70761624999904, -38.6666387115971, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(73, 0, 5.46919958333237, -38.3701989544092, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(74, 0, 5.22244958333238, -38.0737591972212, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(75, 0, 4.96736624999904, -37.7773194400332, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(76, 0, 4.70394958333237, -37.4808796828452, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(77, 0, 4.43219958333237, -37.1844399256572, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(78, 0, 4.15211624999904, -36.8880001684692, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(79, 0, 3.86369958333238, -36.5915604112813, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(80, 0, 3.56694958333238, -36.2951206540933, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(81, 0, 3.26186624999905, -35.9986808969053, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(82, 0, 2.94844958333239, -35.7022411397173, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(83, 0, 2.6266995833324, -35.4058013825293, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(84, 0, 2.29661624999908, -35.1093616253414, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(85, 0, 1.95819958333242, -34.8129218681534, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(86, 0, 1.61144958333241, -34.5164821109654, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(87, 0, 1.25636624999907, -34.2200423537774, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(88, 0, 1, -33.9248109299228, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(89, 0, 1, -33.6504545060682, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(90, 0, 1, -33.4038758599914, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(91, 0, 1, -33.1850749916924, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(92, 0, 1, -32.9940519011711, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(93, 0, 1, -32.8308065884277, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(94, 0, 1, -32.695339053462, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(95, 0, 1, -32.5876492962741, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(96, 0, 1, -32.5077373168639, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(97, 0, 1, -32.4556031152316, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(98, 0, 1, -32.431246691377, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(99, 0, 1, -32.4346680453002, 0, 30, -30, 0, false));


            calcMoves.Add(new calcMove(0, 0, 1, -43.5, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(1, 0, 1, -43.5140277777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(2, 0, 1, -43.5558333333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(3, 0, 1, -43.6254166666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(4, 0, 1, -43.7227777777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(5, 0, 1, -43.8479166666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(6, 0, 1, -44.0008333333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(7, 0, 1, -44.1815277777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(8, 0, 1, -44.39, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(9, 0, 1, -44.62625, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(10, 0, 1, -44.8902777777778, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(11, 0, 1, -45.1820833333333, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(12, 0, 1, -45.5016666666667, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(13, 0, 1, -45.8479616160328, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(14, 0, 1, -46.201515006626, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(15, 0, 1, -46.5550683972192, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(16, 0, 1, -46.9086217878125, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(17, 0, 1.01814601856092, -47.2688025227822, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(18, 0, 1.10659273636713, -47.644211870445, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(19, 0, 1.27723679066161, -48.0159167901704, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(20, 0, 1.53311690819816, -48.3595050551528, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(21, 0, 1.8682455848318, -48.6489790610397, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(22, 0, 2.26693206549991, -48.860809340961, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(23, 0, 2.69870143796323, -48.9771745437656, 0, 30, -30, 15, false));
            calcMoves.Add(new calcMove(24, 0, 3.1747534464523, -48.7484705583286, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(25, 0, 3.63542042598773, -48.5667356558503, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(26, 0, 4.08775407218983, -48.3850007533719, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(27, 0, 4.5317543850586, -48.2032658508936, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(28, 0, 4.96742136459404, -48.0215309484153, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(29, 0, 5.39475501079614, -47.839796045937, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(30, 0, 5.81375532366492, -47.6580611434586, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(31, 0, 6.22442230320036, -47.4763262409803, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(32, 0, 6.62675594940247, -47.294591338502, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(33, 0, 7.02075626227124, -47.1128564360236, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(34, 0, 7.40642324180669, -46.9311215335453, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(35, 0, 7.78375688800881, -46.749386631067, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(36, 0, 8.15275720087755, -46.5676517285887, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(37, 0, 8.51342418041291, -46.3859168261103, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(38, 0, 8.86575782661494, -46.204181923632, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(39, 0, 8.99844958333329, -46.0224470211537, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(40, 0, 8.98919958333322, -45.8407121186754, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(41, 0, 8.97161624999981, -45.658977216197, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(42, 0, 8.94569958333307, -45.4772423137187, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(43, 0, 8.911449583333, -45.2955074112404, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(44, 0, 8.86886624999959, -45.1137725087621, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(45, 0, 8.81794958333285, -44.9320376062837, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(46, 0, 8.75869958333278, -44.7503027038054, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(47, 0, 8.69111624999938, -44.5685678013271, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(48, 0, 8.61519958333264, -44.3868328988488, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(49, 0, 8.53094958333257, -44.2050979963704, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(50, 0, 8.43836624999917, -44.0233630938921, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(51, 0, 8.33744958333243, -43.8416281914138, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(52, 0, 8.22819958333236, -43.6598932889355, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(53, 0, 8.11061624999896, -43.4781583864571, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(54, 0, 7.98469958333223, -43.2964234839788, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(55, 0, 7.85044958333225, -43.1146885815005, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(56, 0, 7.70786624999893, -42.9329536790222, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(57, 0, 7.55694958333228, -42.7512187765438, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(58, 0, 7.3976995833323, -42.5694838740655, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(59, 0, 7.23011624999898, -42.3877489715872, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(60, 0, 7.05419958333232, -42.2060140691089, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(61, 0, 6.86994958333233, -42.0242791666305, 0, 30, -30, 0, false));
            calcMoves.Add(new calcMove(62, 0, 6.67736624999901, -41.8425442641522, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(63, 0, 6.47644958333235, -41.6608093616739, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(64, 0, 6.26719958333236, -41.4790744591956, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(65, 0, 6.04961624999903, -41.2973395567172, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(66, 0, 5.82369958333237, -41.1156046542389, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(67, 0, 5.58944958333237, -40.9338697517606, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(68, 0, 5.34686624999904, -40.7521348492823, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(69, 0, 5.09594958333238, -40.5703999468039, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(70, 0, 4.83669958333237, -40.3886650443256, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(71, 0, 4.56911624999904, -40.2069301418473, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(72, 0, 4.29319958333237, -40.025195239369, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(73, 0, 4.00894958333237, -39.8434603368906, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(74, 0, 3.71636624999904, -39.6617254344123, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(75, 0, 3.41544958333238, -39.479990531934, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(76, 0, 3.10619958333239, -39.2982556294557, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(77, 0, 2.78861624999906, -39.1165207269773, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(78, 0, 2.46269958333241, -38.934785824499, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(79, 0, 2.12844958333242, -38.7530509220207, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(80, 0, 1.78586624999908, -38.5713160195424, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(81, 0, 1.43494958333241, -38.389581117064, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(82, 0, 1.07569958333241, -38.2078462145857, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(83, 0, 1, -38.0173335343298, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(84, 0, 1, -37.7996264096296, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(85, 0, 1, -37.5541415071516, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(86, 0, 1, -37.2808788268958, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(87, 0, 1, -36.9798383688622, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(88, 0, 1, -36.6510201330508, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(89, 0, 1, -36.2995842861559, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(90, 0, 1, -35.9460308955627, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(91, 0, 1, -35.5924775049694, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(92, 0, 1, -35.2389241143762, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(93, 0, 1, -34.885370723783, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(94, 0, 1, -34.5318173331898, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(95, 0, 1, -34.1782639425966, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(96, 0, 1, -33.8247105520033, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(97, 0, 1, -33.4711571614101, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(98, 0, 1, -33.1176037708169, 0, 30, 30, 0, false));
            calcMoves.Add(new calcMove(99, 0, 1, -32.7640503802237, 0, 30, 30, 0, false));




        }

        public string CustomRendering()
        {
            return "";
        }
    }
}
