import json
import robotic as ry

markers = list(range(9))
positions = dict()

C = ry.Config()
C.addFile(ry.raiPath("scenarios/pandaSingle.g"))

bot = ry.BotOp(C, True)
for id in markers:
    bot.hold(floating=True, damping=False)
    print(f"Move Gripper to marker {id}, then press q")
    while bot.getKeyPressed() != ord('q'):
        bot.sync(C, viewMsg=f"Move Gripper to marker {id}, then press q")

    bot.sync(C)
    bot.hold()
    pos = C.eval(ry.FS.positionRel, ['l_gripper', 'l_panda_base'])[0]
    positions[id] = pos.tolist()
    bot.sync(C)

with open('marker_positions.json', 'w') as f:
    json.dump(positions, f, indent=4)