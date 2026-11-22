import AppKit
let width = 1600, height = 1120
let bitmap = NSBitmapImageRep(bitmapDataPlanes: nil, pixelsWide: width, pixelsHigh: height, bitsPerSample: 8, samplesPerPixel: 4, hasAlpha: true, isPlanar: false, colorSpaceName: .deviceRGB, bytesPerRow: 0, bitsPerPixel: 0)!
NSGraphicsContext.saveGraphicsState()
NSGraphicsContext.current = NSGraphicsContext(bitmapImageRep: bitmap)
let ctx = NSGraphicsContext.current!.cgContext
ctx.translateBy(x: 0, y: CGFloat(height)); ctx.scaleBy(x: 1, y: -1)
func color(_ hex: Int) -> NSColor { NSColor(red: CGFloat((hex >> 16) & 255)/255, green: CGFloat((hex >> 8) & 255)/255, blue: CGFloat(hex & 255)/255, alpha: 1) }
func text(_ s: String, _ x: CGFloat, _ y: CGFloat, _ size: CGFloat = 22, _ bold: Bool = false, _ ink: Int = 0x19324B) {
    let font = bold ? NSFont.boldSystemFont(ofSize: size) : NSFont.systemFont(ofSize: size)
    let attrs: [NSAttributedString.Key: Any] = [.font: font, .foregroundColor: color(ink)]
    let str = NSAttributedString(string: s, attributes: attrs)
    ctx.saveGState(); ctx.translateBy(x: x, y: y + size * 1.25); ctx.scaleBy(x: 1, y: -1); str.draw(at: .zero); ctx.restoreGState()
}
func box(_ x: CGFloat, _ y: CGFloat, _ w: CGFloat, _ h: CGFloat, _ title: String, _ lines: [String], _ fill: Int = 0xE9F2FA) {
    color(fill).setFill(); NSBezierPath(roundedRect: NSRect(x:x,y:y,width:w,height:h), xRadius:14,yRadius:14).fill()
    text(title,x+20,y+14,24,true)
    for (i,line) in lines.enumerated() { text(line,x+20,y+52+CGFloat(i)*29,20) }
}
func arrow(_ points: [(CGFloat,CGFloat)], _ ink: Int = 0x3877A6) {
    ctx.setStrokeColor(color(ink).cgColor); ctx.setFillColor(color(ink).cgColor); ctx.setLineWidth(3)
    ctx.move(to: CGPoint(x:points[0].0,y:points[0].1)); for p in points.dropFirst(){ctx.addLine(to:CGPoint(x:p.0,y:p.1))}; ctx.strokePath()
    let a=points[points.count-2], b=points.last!, angle=atan2(b.1-a.1,b.0-a.0)
    ctx.move(to: CGPoint(x:b.0,y:b.1)); ctx.addLine(to:CGPoint(x:b.0-13*cos(angle-0.45),y:b.1-13*sin(angle-0.45))); ctx.addLine(to:CGPoint(x:b.0-13*cos(angle+0.45),y:b.1-13*sin(angle+0.45))); ctx.closePath();ctx.fillPath()
}
color(0xFFFFFF).setFill();ctx.fill(CGRect(x:0,y:0,width:width,height:height))
text("CANARD BOARD  /  CODEBASE MAP",50,30,34,true)
text("Read the center flow first. The flight state machine decides what can run.",50,81,23)
box(50,140,1500,125,"1  STARTUP",["main.c initializes STM32 hardware, then creates the system initialization task.","init.c initializes modules and creates FreeRTOS tasks; the scheduler runs them by priority."],0xEDF0F5)
arrow([(800,265),(800,305)])
box(50,310,1500,95,"2  FSM COORDINATOR  •  application/fsm/fsm.c",["Timer notification → collect measurements → consume one flight event → update state → run state actions"],0xDCECF9)
box(50,465,320,160,"Sensor drivers",["Board IMU, magnetometer,","barometer, Movella, AD board","I2C / UART / ADC"],0xE4F4EF)
box(420,465,330,160,"Sensor handler",["Read latest measurements","Apply units + calibration","Mark new measurements"],0xE4F4EF)
box(800,465,330,160,"Navigator",["Estimate attitude, velocity,","altitude and roll state","Uses generated GNC code"],0xE4F4EF)
box(1180,465,370,160,"Controller → motor",["Roll state + motor feedback","Compute canard angle","AK45 driver sends command"],0xE4F4EF)
arrow([(370,545),(420,545)]);arrow([(750,545),(800,545)]);arrow([(1130,545),(1180,545)])
text("Physical measurements",65,430,18);text("State estimate",830,430,18);text("Control: 10 ms minimum interval*",1180,430,18)
box(50,680,475,160,"Flight phase + CAN commands",["Events from commands, acceleration, timers","IDLE → pad preparation → BOOST","→ ACT_ALLOWED → RECOVERY → SLEEPY"],0xFFF1DC)
box(560,680,475,160,"Telemetry + logger",["Telemetry schedules sources by flight phase","CAN TX/RX connects to the rocket network","Logger writes data and messages to SD"],0xEDF0F5)
box(1070,680,480,160,"Health + power",["Checks module and board status every 1 s","Reports errors over CAN; fatal path resets","Watchdog API exists; no task callers found"],0xFCE9EA)
arrow([(50,730),(25,730),(25,355),(50,355)],0xB47B2A)
box(50,885,1500,155,"3  WHERE TO READ NEXT",["application/ = behavior and coordination     drivers/ = device access     common/ = shared types and math","third_party/ = STM32 HAL + FreeRTOS + rocketlib + generated GNC algorithms","tests/ = host-side GoogleTest with hardware/RTOS mocks     HIL = Simulink supplies simulated sensors"],0xEDF0F5)
text("* Controller requires fresh motor feedback. Motor command is forced to zero in pad/boost states;",50,1055,18)
text("  computed commands are sent in ACT_ALLOWED and RECOVERY. This is a source-level map, not hardware validation.",50,1080,18)
NSGraphicsContext.restoreGraphicsState()
let url = URL(fileURLWithPath: CommandLine.arguments.count > 1 ? CommandLine.arguments[1] : "docs/codebase-overview/architecture.png")
try bitmap.representation(using: .png, properties: [:])!.write(to: url)
