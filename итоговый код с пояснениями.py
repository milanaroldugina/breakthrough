
#!/usr/bin/env python3 - система запуска
"""ПРОРЫВ. Menu  → игра → PLAY AGAIN / MENU."""

print("START")
import math, socket, time, heapq
import cv2
import numpy as np

CAMERA_INDEX = 1 #индекс камеры. 0 - как правило встроенная
ROBOT_MARKER_ID = 3 #айди используемых ArUco маркеров
GOAL_MARKER_ID = 0
ROBOT_ADDRESS = ("192.168.4.1", 8888) #IP и порт робота 
ARUCO_DICT = cv2.aruco.DICT_4X4_50 #словарь маркеров 4х4

LINEAR_SPEED = 350 #скорости робота вперед и поворот
ANGULAR_SPEED_MRAD = 4000
GRID_CELL = 20 #размер ячейки сетки для А*
BRUSH = 12 #размер кисти для рисования стенок
SAFETY = 35 #Буфер безопасности — на сколько пикселей расширяем стены, чтобы робот не задел их краем.
MELT_EVERY = 0.15 #игровые характеристики (таяние стен, запас чернил, длительность раунда)
INK_BUDGET_PX = 2500
DUEL_SEC = 30
REPLAN = 0.1
HUD = 30
HOME_PX = 50
WIN = "BREAKTHROUGH"

COL_ROBOT = (0, 140, 255) #палитры цветов
COL_GOAL = (0, 0, 230) 
COL_PATH = (0, 140, 255)
COL_WALL = (230, 150, 50)
COL_INK = (230, 150, 50)
COL_BTN = (0, 160, 230)


def signed_angle(heading, vector): #знаковый угол между направлением робота (heading) и вектором к цели (vector) в радианах
    hx, hy = float(heading[0]), -float(heading[1])
    vx, vy = float(vector[0]), -float(vector[1])
    return math.atan2(hx * vy - hy * vx, hx * vx + hy * vy)


def marker_geometry(corners): #вычисление центра маркера, вектор направления и размер в пикселя (для общего масштаба)
    pts = corners.reshape(4, 2)
    center = pts.mean(axis=0)
    heading = 0.5 * (pts[0] + pts[1]) - center
    heading /= max(float(np.linalg.norm(heading)), 1.0)
    size = float(np.linalg.norm(pts[1] - pts[0]))
    return center.astype(np.float32), heading.astype(np.float32), size


def in_rect(x, y, rect): #для кликов по кнопкам
    rx, ry, rw, rh = rect
    return rx <= x <= rx + rw and ry <= y <= ry + rh


class Link: #соединение с роботом
    def __init__(s):
        s.s = None; s.ok = False; s.last_try = 0.0
    def _connect(s): #GET запросы
        now = time.monotonic()
        if s.s is None and now - s.last_try > 3.0:
            s.last_try = now
            try:
                s.s = socket.create_connection(ROBOT_ADDRESS, timeout=1.0)
                s.s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.s.settimeout(1.0)
                s.s.sendall(b"GET\n")
                s.ok = True
                print(f"  Link: connected {ROBOT_ADDRESS}")
            except OSError as e:
                print(f"  Link: no robot ({e})")
                s.s = None; s.ok = False
    def send(s, cmd): #отправка команды роботу
        s._connect()
        if s.s is None: return
        try:
            s.s.sendall((cmd + "\n").encode()); s.ok = True
        except OSError:
            s.ok = False; s.s = None
    def vel(s, l, a): s.send(f"VEL {int(l)} {int(a)}") #скорости
    def stop(s): s.send("STOP") #стоп
    def close(s): #завершение связи с роботом
        if s.s:
            try: s.s.close()
            except: pass


def make_detector(): #точный поиск маркеров 
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(ARUCO_DICT), p)


def inflate(mask, r): #расширение маски
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    return cv2.dilate(mask, k)


def blocked_from_mask(mask): #перевод стенок в заблокированные зоны
    h, w = mask.shape
    rows, cols = math.ceil(h / GRID_CELL), math.ceil(w / GRID_CELL)
    blocked = set()
    for r in range(rows):
        y0, y1 = r * GRID_CELL, min(h, (r + 1) * GRID_CELL)
        for c in range(cols):
            x0, x1 = c * GRID_CELL, min(w, (c + 1) * GRID_CELL)
            if np.any(mask[y0:y1, x0:x1] > 0): blocked.add((r, c))
    return blocked, rows, cols


def astar(start, goal, blocked, rows, cols): #А* алгоритм - кратчайший путь по сетке
    if goal in blocked: return None
    blocked = set(blocked); blocked.discard(start)
    heap = [(math.hypot(start[0] - goal[0], start[1] - goal[1]), 0.0, start)]
    came, best = {}, {start: 0.0}
    while heap:
        _, cost, cur = heapq.heappop(heap)
        if cur == goal:
            path = [cur]
            while cur in came: cur = came[cur]; path.append(cur)
            return path[::-1]
        if cost > best.get(cur, float("inf")): continue
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0: continue
                n = (cur[0] + dr, cur[1] + dc)
                if not (0 <= n[0] < rows and 0 <= n[1] < cols) or n in blocked: continue
                if dr and dc and ((cur[0] + dr, cur[1]) in blocked or (cur[0], cur[1] + dc) in blocked): continue
                nc = cost + (math.sqrt(2) if dr and dc else 1.0)
                if nc >= best.get(n, float("inf")): continue
                best[n] = nc; came[n] = cur
                heapq.heappush(heap, (nc + math.hypot(n[0] - goal[0], n[1] - goal[1]), nc, n))
    return None


def plan_path(robot_px, goal_px, obstacle_mask): #ячейки в пиксели иобратно, сравнение первого и последнего пунктов цели (точно ли робот ее достиг)
    h, w = obstacle_mask.shape
    inflated = inflate(obstacle_mask, SAFETY)
    planning = inflated.copy()
    rp = tuple(np.rint(robot_px).astype(int))
    esc = np.zeros_like(planning)
    cv2.circle(esc, rp, SAFETY + GRID_CELL, 255, -1)
    planning[(esc > 0) & (obstacle_mask == 0)] = 0
    blocked, rows, cols = blocked_from_mask(planning)
    def cell(p):
        return (max(0, min(rows - 1, int(p[1]) // GRID_CELL)),
                max(0, min(cols - 1, int(p[0]) // GRID_CELL)))
    cells = astar(cell(robot_px), cell(goal_px), blocked, rows, cols)
    if cells is None: return None, inflated
    pts = [np.array([min(w - 1, c * GRID_CELL + GRID_CELL / 2),
                     min(h - 1, r * GRID_CELL + GRID_CELL / 2)], np.float32)
           for r, c in cells]
    pts[0] = robot_px.astype(np.float32); pts[-1] = goal_px.astype(np.float32)
    return pts, inflated


def draw_button(frame, rect, text, color=COL_BTN, text_color=(255, 255, 255), scale=0.8, thick=2): #отрисовка кнопки
    x, y, w, h = rect
    cv2.rectangle(frame, (x, y), (x + w, y + h), color, -1)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 255), 2)
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    cv2.putText(frame, text, (x + (w - tw) // 2, y + (h + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, scale, text_color, thick)


def draw_centered_text(frame, text, y, scale, color, thick=2): #текст по центру экрана
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    cv2.putText(frame, text, ((frame.shape[1] - tw) // 2, y),
                cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick)


class Arena: #основная логика игры
    def __init__(s):
        s.cam = cv2.VideoCapture(CAMERA_INDEX)
        s.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        s.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 600) #открываем камеру
        s.det = make_detector() #детектор маркеров и связь с роботом
        s.bot = Link()
        s.mask = None #чб маска стен, потраченные чернила, длина штриха
        s.ink_spent = 0.0
        s.cur_len = None
        s.last_x = 0; s.last_y = 0
        s.path = []; s.wp = 0 #список точек маршрута
        s.phase = "menu"  #финал, время отсчета
        s.fin = None
        s.home = None
        s.cd0 = 0.0
        s.mode = "duel"; s.t0 = 0.0; s.time_left = DUEL_SEC #режим дуэль (планируется добавлять другие версии), время оставшееся и старта
        s.robot_c = None; s.robot_h = None; s.robot_size = 45 #робот (центр, направление, размер)
        s.goal_c = None; s.goal_size = 30; s.goal_manual = False #ворота (центр, направление, размер)
        s.last_plan = 0.0; s.map_changed = True #флаги рисовки мышью
        s.last_robot_seen = 0.0
        s.drawing = False
        s.msg = ""
        s.topmost = False
        s.fw = 800; s.fh = 600
        s.is_restart = False
        cv2.namedWindow(WIN)#обработчик мыши
        cv2.setMouseCallback(WIN, s.mouse)

    def win_px(s): #радиус победы
        return max(60.0, (s.goal_size + s.robot_size) * 0.7)

    def ink_used(s): #сколько чернил использовано
        return s.ink_spent + (s.cur_len or 0)

    def menu_btn(s): #координаты кнопки play
        return (s.fw // 2 - 120, s.fh // 2 + 80, 240, 60)

    def end_btns(s): #координаты menu and restart
        return (s.fw // 2 - 260, s.fh // 2 + 60, 220, 55), (s.fw // 2 + 40, s.fh // 2 + 60, 220, 55)

    def can_draw(s, x, y): #можно ли рисовать? (нельзя когда ближе 40 пикселей к роботу, радиус победы к воротам, при отсутствии чернил)
        if y < HUD: return False
        if s.ink_used() >= INK_BUDGET_PX: return False
        if s.robot_c is not None and math.hypot(x - s.robot_c[0], y - s.robot_c[1]) < 40: return False
        if s.goal_c is not None and math.hypot(x - s.goal_c[0], y - s.goal_c[1]) < s.win_px(): return False
        return True

    def mouse(s, ev, x, y, fl, *rest): #обработчик мыши - запуск игры, все кнопки
        if ev == cv2.EVENT_LBUTTONDOWN:
            if s.phase == "menu":
                if in_rect(x, y, s.menu_btn()):
                    s.on_play()
                return
            if s.phase == "finished":
                btn_menu, btn_play = s.end_btns()
                if in_rect(x, y, btn_play):
                    s.is_restart = True
                    s.on_play()
                elif in_rect(x, y, btn_menu):
                    s.phase = "menu"; s.fin = None; s.msg = ""
                    s.bot.stop()
                return
            if s.mask is None: return
            if s.can_draw(x, y):
                s.drawing = True
                s.cur_len = 0.0
                s.last_x, s.last_y = x, y
                cv2.circle(s.mask, (x, y), BRUSH, 255, -1); s.map_changed = True
        elif ev == cv2.EVENT_MOUSEMOVE and s.drawing and (fl & cv2.EVENT_FLAG_LBUTTON):
            if s.mask is None: return
            if s.can_draw(x, y) and s.cur_len is not None:
                seg = math.hypot(x - s.last_x, y - s.last_y)
                s.last_x, s.last_y = x, y
                if seg < 2: return
                if s.ink_used() + seg > INK_BUDGET_PX: return
                cv2.circle(s.mask, (x, y), BRUSH, 255, -1)
                s.cur_len += seg; s.map_changed = True
        elif ev == cv2.EVENT_LBUTTONUP:
            s.drawing = False
            if s.cur_len is not None:
                s.ink_spent += s.cur_len
            s.cur_len = None
        elif ev == cv2.EVENT_RBUTTONDOWN and s.phase not in ("menu", "finished"):
            if s.mask is not None:
                s.goal_c = np.array([x, y], np.float32)
                s.goal_manual = True
                s.msg = "GOAL SET"

    def on_play(s): #старт пи обоих видных маркерах (иначе можно ПКМ их обозначить на карте)
        if s.robot_c is not None and s.goal_c is not None:
            s.start_game()
        else:
            s.phase = "idle"
            s.msg = "SHOW MARKERS (3 & 0) OR RMB"

    def start_game(s): #очищает стены, чернила, маршрут. При рестарте - возврат на позицию
        if s.robot_c is None or s.goal_c is None:
            s.msg = "NEED MARKERS"
            return
        s.mask.fill(0); s.ink_spent = 0.0; s.cur_len = None
        s.path = []; s.wp = 0; s.map_changed = True
        s.fin = None; s.msg = ""
        s.last_plan = 0.0
        
        if s.is_restart and s.home is not None:
            s.phase = "return"
            s.msg = "RETURN TO START"
        else:
            s.phase = "countdown"
            s.cd0 = time.monotonic()
            s.msg = "GET READY"

    def steer(s, target, slow_dist=0): #руление к точке
        vec = target - s.robot_c
        dist = float(np.linalg.norm(vec))
        err = signed_angle(s.robot_h, vec)
        if abs(err) > 2.4:
            s.bot.vel(0, ANGULAR_SPEED_MRAD if err > 0 else -ANGULAR_SPEED_MRAD)
            s.msg = "TURNING"
        else:
            lin = LINEAR_SPEED * max(0.3, math.cos(err))
            if slow_dist > 0:
                lin *= max(0.3, min(1.0, dist / slow_dist))
            ang = max(-ANGULAR_SPEED_MRAD, min(ANGULAR_SPEED_MRAD, err * 4000))
            s.bot.vel(lin, ang)
            s.msg = "RETURNING"
        return dist

    def control(s): #если нет пути - едем к цели прямо
        if not s.path:
            err = signed_angle(s.robot_h, s.goal_c - s.robot_c)
            if abs(err) > 2.4:
                s.bot.vel(0, ANGULAR_SPEED_MRAD if err > 0 else -ANGULAR_SPEED_MRAD)
                s.msg = "TURNING"
            else:
                s.bot.vel(LINEAR_SPEED, 0)
                s.msg = "DRIVING"
            return
        
        wp = s.path[s.wp] if s.wp < len(s.path) else s.path[-1]
        err = signed_angle(s.robot_h, wp - s.robot_c)
        if abs(err) > 2.4:
            s.bot.vel(0, ANGULAR_SPEED_MRAD if err > 0 else -ANGULAR_SPEED_MRAD)
            s.msg = "TURNING"
        else:
            lin = LINEAR_SPEED * max(0.3, math.cos(err))
            ang = max(-ANGULAR_SPEED_MRAD, min(ANGULAR_SPEED_MRAD, err * 4000))
            s.bot.vel(lin, ang)
            s.msg = "DRIVING"
        
        if float(np.linalg.norm(wp - s.robot_c)) < 25:
            s.wp += 1

    def run(s): #читает первый кадр, запоминает размеры, создаёт пустую маску стен.
        ok, first = s.cam.read()
        if not ok: print("CAMERA FAILED"); return
        h, w = first.shape[:2]
        s.fw, s.fh = w, h
        s.mask = np.zeros((h, w), np.uint8)
        melt_t = 0.0

        while True: #захватывает кадр, получает время
            s.cam.grab()
            ok, frame = s.cam.read()
            if not ok: break
            now = time.monotonic()

            melt_t += 0.03 #таяние стен
            if melt_t >= MELT_EVERY:
                melt_t = 0.0
                if np.any(s.mask > 0):
                    s.mask = cv2.erode(s.mask, np.ones((3, 3), np.uint8))
                    s.map_changed = True

            corners, ids, _ = s.det.detectMarkers(frame) #детекция маркеров
            if ids is not None:
                for c, mid in zip(corners, ids.flatten()):
                    ctr, hd, sz = marker_geometry(c)
                    if int(mid) == GOAL_MARKER_ID and not s.goal_manual:
                        s.goal_c, s.goal_size = ctr, sz
                    elif int(mid) == ROBOT_MARKER_ID:
                        s.robot_c, s.robot_h, s.robot_size = ctr, hd, sz
                        s.last_robot_seen = now
            if s.home is None and s.robot_c is not None: #запоминаем стартовую позицию
                s.home = s.robot_c.copy()

            #фазы
            if s.phase == "menu": #меню
                pass
            elif s.phase == "idle": #ожидание маркеров
                if s.robot_c is None: s.msg = "SHOW ROBOT (3)"
                elif s.goal_c is None: s.msg = "SHOW GOAL (0)"
                else: s.msg = "PRESS SPACE"
            elif s.phase == "return": #возврат на старт позицию
                # Робот едет на стартовую позицию
                if s.robot_c is None:
                    s.bot.stop(); s.phase = "idle"; s.msg = "LOST ROBOT"
                else:
                    d = s.steer(s.home, slow_dist=150)
                    if d < HOME_PX:
                        s.bot.stop()
                        s.phase = "countdown"
                        s.cd0 = time.monotonic()
                        s.msg = "GET READY"
            elif s.phase == "countdown":
                # Робот СТОИТ, ждёт отсчёта
                s.bot.stop()
                if now - s.cd0 >= 3.0:
                    s.phase = "running"
                    s.t0 = now
                    s.time_left = DUEL_SEC
                    s.last_plan = 0.0
                    s.msg = "GO"
            elif s.phase == "running": #гонка, каждые 0.1 сек перестраиваем путь, для скорости рулим по пути
                if s.robot_c is not None and s.goal_c is not None:
                    if now - s.last_plan >= REPLAN or s.map_changed:
                        path, _ = plan_path(s.robot_c, s.goal_c, s.mask)
                        s.last_plan = now
                        s.map_changed = False
                        if path is not None:
                            s.path = path
                            s.wp = 1 if len(path) > 1 else 0
                    
                    s.control()
                    
                    dist = float(np.linalg.norm(s.goal_c - s.robot_c))
                    if s.mode == "duel":
                        s.time_left = max(0.0, DUEL_SEC - (now - s.t0))
                    
                    if dist <= s.win_px():
                        s.bot.stop(); s.phase = "finished"
                        s.fin = "ROBOT WIN!"
                    elif s.mode == "duel" and s.time_left <= 0:
                        s.bot.stop(); s.phase = "finished"
                        s.fin = "YOU WIN!"
            elif s.phase == "finished": #в конце робот стоит
                s.bot.stop()

            #отрисовка
            if s.phase == "menu": #затемнение фона, заголовки, кнопки
                overlay = frame.copy()
                cv2.rectangle(overlay, (0, 0), (w, h), (0, 0, 0), -1)
                frame = cv2.addWeighted(frame, 0.3, overlay, 0.7, 0)
                draw_centered_text(frame, "ПРОРЫВ", h // 2 - 60, 2.2, COL_ROBOT, 4)
                draw_centered_text(frame, "BREAKTHROUGH", h // 2 - 10, 0.9, (255, 255, 255), 2)
                draw_button(frame, s.menu_btn(), "▶  PLAY", scale=1.0, thick=3)
                draw_centered_text(frame, "SPACE / ENTER / R = start", h - 30, 0.4, (150, 150, 150), 1)
            else: #белый кружок старт позиции
                if s.home is not None:
                    hm = tuple(np.rint(s.home).astype(int))
                    cv2.circle(frame, hm, 8, (255, 255, 255), 2)
                
                if np.any(s.mask > 0): #стены
                    layer = np.zeros_like(frame); layer[:] = COL_WALL
                    mb = s.mask > 0
                    frame[mb] = cv2.addWeighted(frame, 0.5, layer, 0.5, 0)[mb]
                
                if s.goal_c is not None: #ворота
                    gc = tuple(np.rint(s.goal_c).astype(int))
                    draw_r = int(s.win_px() * 0.6)
                    cv2.circle(frame, gc, draw_r, COL_GOAL, 2)
                
                if len(s.path) >= 2 and s.phase == "running": #маршрут
                    cv2.polylines(frame, [np.rint(s.path).astype(np.int32)], False, COL_PATH, 2)
                
                if s.robot_c is not None and s.robot_h is not None: #робот и вектор
                    rc = tuple(np.rint(s.robot_c).astype(int))
                    cv2.circle(frame, rc, 6, COL_ROBOT, -1)
                    cv2.arrowedLine(frame, rc, tuple(np.rint(s.robot_c + s.robot_h * 35).astype(int)), COL_ROBOT, 2)
                
                if s.phase == "countdown": #обратный отсчет
                    n = max(1, 3 - int(now - s.cd0))
                    draw_centered_text(frame, str(n), h // 2 + 40, 3.0, COL_ROBOT, 6)
                
                cv2.rectangle(frame, (0, 0), (w, HUD), (0, 0, 0), -1) #черная полоса как меню сверху (время, кол во чернил...)
                frac = max(0.0, 1 - s.ink_used() / INK_BUDGET_PX)
                cv2.rectangle(frame, (10, 10), (110, 20), (50, 50, 50), -1)
                cv2.rectangle(frame, (10, 10), (int(10 + 100 * frac), 20), COL_INK, -1)
                cv2.putText(frame, "INK", (115, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.putText(frame, f"MODE: {s.mode.upper()}", (160, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, COL_ROBOT, 1)
                link_color = (100, 255, 100) if s.bot.ok else (80, 80, 255)
                cv2.putText(frame, f"LINK: {'OK' if s.bot.ok else 'OFF'}", (260, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, link_color, 1)
                if s.mode == "duel" and s.phase in ("running", "finished"):
                    time_color = (80, 80, 255) if s.time_left < 10 else (255, 255, 255)
                    cv2.putText(frame, f"TIME: {int(s.time_left)}s", (360, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, time_color, 1)
                cv2.putText(frame, f"STATUS: {s.msg}", (460, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                if s.fin: #законченый раунд - результаты
                    draw_centered_text(frame, s.fin, h // 2 - 30, 1.0, COL_ROBOT, 3)
                    btn_menu, btn_play = s.end_btns()
                    draw_button(frame, btn_menu, "MENU", color=(80, 80, 80), text_color=(180, 180, 180))
                    draw_button(frame, btn_play, "RESTART")

            cv2.imshow(WIN, frame) #обновление картинки на экране
            if not s.topmost:
                cv2.setWindowProperty(WIN, cv2.WND_PROP_TOPMOST, 1)
                s.topmost = True
            k = cv2.waitKey(30) & 0xFF #обработка клавиш
            if k == 27: break
            if k in (13, 10, 32, 114, 82):
                if s.phase == "menu":
                    s.is_restart = False
                    s.on_play()
                elif s.phase == "finished":
                    s.is_restart = True
                    s.on_play()
                elif s.phase == "idle" and s.robot_c is not None and s.goal_c is not None:
                    s.is_restart = False
                    s.start_game()
            if k == 8:
                s.bot.stop()
                s.mask.fill(0); s.ink_spent = 0.0; s.cur_len = None
                s.path = []; s.fin = None
                s.goal_manual = False
                s.phase = "idle"
                s.map_changed = True; s.msg = "RESET"

        s.bot.stop(); s.bot.close(); s.cam.release(); cv2.destroyAllWindows() #завершение


if __name__ == "__main__": #вход и запуск
    try:
        Arena().run()
    except Exception as e:
        print(f"!! ERROR: {e}")
        import traceback; traceback.print_exc()
