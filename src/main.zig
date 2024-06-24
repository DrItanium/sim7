const std = @import("std");
const meta = @import("std").meta;
const expect = std.testing.expect;
const Operand = u5;

const ByteInteger = i8;
const ByteOrdinal = u8;
const ShortInteger = i16;
const ShortOrdinal = u16;
const Integer = i32;
const Ordinal = u32;
const LongOrdinal = u64;
const LongInteger = i64;
const TripleOrdinal = u96;
const QuadOrdinal = u128;
const Address = u32;

const Real = f32;
const LongReal = f64;
const ExtendedReal = f80;
fn StorageFrame(
    comptime T: type,
    comptime count: comptime_int,
) type {
    return [count]T;
}

const RegisterFrame = StorageFrame(Ordinal, 16);

const ArchitectureLevel = enum {
    Core,
    Numerics,
    Protected,
    Extended,
};
const InstructionClass = enum(u2) {
    CTRL,
    COBR,
    REG,
    MEM,
};

fn determineInstructionClass(opcode: u8) InstructionClass {
    return switch (opcode) {
        0x00...0x1F => InstructionClass.CTRL,
        0x20...0x3F => InstructionClass.COBR,
        0x40...0x7F => InstructionClass.REG,
        0x80...0xFF => InstructionClass.MEM,
    };
}
const DecodedOpcode = enum(u12) {
    b = 0x8,
    call,
    ret,
    bal,
    bno,
    bg,
    be,
    bge,
    bl,
    bne,
    ble,
    bo,
    faultno,
    faultg,
    faulte,
    faultge,
    faultl,
    faultne,
    faultle,
    faulto,
    testno,
    testg,
    teste,
    testge,
    testl,
    testne,
    testle,
    testo,
    bbc = 0x30,
    cmpobg,
    cmpobe,
    cmpobge,
    cmpobl,
    cmpobne,
    cmpoble,
    bbs,
    cmpibno,
    cmpibg,
    cmpibe,
    cmpibge,
    cmpibl,
    cmpibne,
    cmpible,
    cmpibo,
    ldob = 0x80,
    ldvob,
    stob = 0x82,
    stvob,
    bx = 0x84,
    balx = 0x85,
    callx = 0x86,
    ldos = 0x88,
    ldvos,
    stos = 0x8a,
    stvos,
    lda = 0x8c,
    ld = 0x90,
    ldv,
    st = 0x92,
    stv,
    ldl = 0x98,
    ldvl,
    stl = 0x9a,
    stvl,
    ldt = 0xa0,
    ldvt,
    stt = 0xa2,
    stvt,
    dcinva = 0xad,
    ldq = 0xb0,
    ldvq,
    stq = 0xb2,
    stvq,
    ldib = 0xc0,
    ldvib,
    stib = 0xc2,
    stvib,
    ldis = 0xc8,
    ldvis,
    stis = 0xca,
    stvis,
    ldm = 0xd0,
    ldvm,
    stm,
    stvm,
    ldml = 0xd8,
    ldvml,
    stml,
    stvml,
    ldmq = 0xf0,
    ldvmq,
    stmq,
    stvmq,
    notbit = 0x580,
    @"and" = 0x581,
    andnot = 0x582,
    setbit = 0x583,
    notand,
    xor = 0x586,
    @"or",
    nor,
    xnor,
    not,
    ornot,
    clrbit,
    notor,
    nand,
    alterbit,
    addo,
    addi,
    subo,
    subi,
    cmpob,
    cmpib,
    cmpos,
    cmpis,
    shro = 0x598,
    shrdi = 0x59a,
    shri,
    shlo,
    rotate,
    shli,
    cmpo = 0x5a0,
    cmpi,
    concmpo,
    concmpi,
    cmpinco,
    cmpinci,
    cmpdeco,
    cmpdeci,
    scanbyte = 0x5ac,
    chkbit = 0x5ae,
    addc = 0x5b0,
    subc = 0x5b2,
    mov = 0x5cc,
    movl = 0x5dc,
    movt = 0x5ec,
    movq = 0x5fc,
    synmov = 0x600,
    synmovl,
    synmovq,
    cmpstr,
    movqstr,
    movstr,
    atmod = 0x610,
    atrep,
    atadd,
    inspacc,
    ldphy,
    synld,
    fill = 0x617,
    spanbit = 0x640,
    scanbit,
    daddc,
    dsubc,
    dmovt,
    modac,
    condrec,
    modify = 0x650,
    extract,
    modtc = 0x654,
    modpc,
    receive,
    calls = 0x660,
    send = 0x662,
    sendserv,
    resumprcs,
    schedprcs,
    saveprcs,
    condwait = 0x668,
    wait,
    signal,
    mark,
    fmark,
    flushreg,
    syncf = 0x66f,
    emul = 0x670,
    ediv,
    ldtime = 0x673,
    cvtir,
    cvtilr,
    scalerl,
    scaler,
    atanr = 0x680,
    logepr,
    logr,
    remr,
    cmpor,
    cmpr,
    sqrtr = 0x688,
    expr,
    logbnr,
    roundr,
    sinr,
    cosr,
    tanr,
    classr,
    atanrl = 0x690,
    logeprl,
    logrl,
    remrl,
    cmporl,
    cmprl,
    sqrtrl = 0x698,
    exprl,
    logbnrl,
    roundrl,
    sinrl,
    cosrl,
    tanrl,
    classrl,
    cvtri = 0x6c0,
    cvtril,
    cvtzri,
    cvtzril,
    movr = 0x6c9,
    movrl = 0x6d9,
    cpysre = 0x6e2,
    cpyrsre,
    movre = 0x6e9,
    mulo = 0x701,
    remo = 0x708,
    divo = 0x70b,
    muli = 0x741,
    remi = 0x748,
    modi,
    divi = 0x74b,
    divr = 0x78b,
    mulr,
    subr,
    addr = 0x78f,
    divrl = 0x79b,
    mulrl,
    subrl,
    addrl = 0x79f,
    // extended instructions
    chktag = 0x5a8,
    cmpm = 0x5aa,
    bswap = 0x5ad,
    intdis = 0x5b4,
    inten = 0x5b5,
    movm = 0x5cd,
    eshro = 0x5d8,
    movml = 0x5dd,
    movmq = 0x5fd,
    cvtadr = 0x672,
    restrict = 0x652,
    amplify = 0x653,
    ldcsp = 0x657,
    intctl = 0x658,
    sysctl = 0x659,
    icctl = 0x65b,
    dcctl = 0x65c,
    halt = 0x65d,
    sdma = 0x630,
    udma = 0x631,
    addono = 0x780,
    addino = 0x781,
    subono = 0x782,
    subino = 0x783,
    selno = 0x784,
    addog = 0x790,
    addig = 0x791,
    subog = 0x792,
    subig = 0x793,
    selg = 0x794,
    addoe = 0x7a0,
    addie = 0x7a1,
    suboe = 0x7a2,
    subie = 0x7a3,
    sele = 0x7a4,
    addoge = 0x7b0,
    addige = 0x7b1,
    suboge = 0x7b2,
    subige = 0x7b3,
    selge = 0x7b4,
    addol = 0x7c0,
    addil = 0x7c1,
    subol = 0x7c2,
    subil = 0x7c3,
    sell = 0x7c4,
    addone = 0x7d0,
    addine = 0x7d1,
    subone = 0x7d2,
    subine = 0x7d3,
    selne = 0x7d4,
    addole = 0x7e0,
    addile = 0x7e1,
    subole = 0x7e2,
    subile = 0x7e3,
    selle = 0x7e4,
    addoo = 0x7f0,
    addio = 0x7f1,
    suboo = 0x7f2,
    subio = 0x7f3,
    selo = 0x7f4,

    pub fn getPrimaryOpcode(self: DecodedOpcode) u8 {
        return switch (self) {
            0x00...0xFF => @intFromEnum(self),
            else => @intFromEnum(self) >> 4,
        };
    }
    pub fn getInstructionClass(self: DecodedOpcode) InstructionClass {
        return comptime determineInstructionClass(self.getPrimaryOpcode());
    }
    pub fn getArchitectureLevel(self: DecodedOpcode) ArchitectureLevel {
        return switch (self) {
            DecodedOpcode.chktag,
            DecodedOpcode.cmpm,
            DecodedOpcode.movm,
            DecodedOpcode.movml,
            DecodedOpcode.movmq,
            DecodedOpcode.cvtadr,
            DecodedOpcode.restrict,
            DecodedOpcode.amplify,
            DecodedOpcode.ldcsp,
            DecodedOpcode.ldvob,
            DecodedOpcode.stvob,
            DecodedOpcode.ldvos,
            DecodedOpcode.stvos,
            DecodedOpcode.ldv,
            DecodedOpcode.stv,
            DecodedOpcode.ldvl,
            DecodedOpcode.stvl,
            DecodedOpcode.ldvt,
            DecodedOpcode.stvt,
            DecodedOpcode.ldvq,
            DecodedOpcode.stvq,
            DecodedOpcode.ldvib,
            DecodedOpcode.stvib,
            DecodedOpcode.ldvis,
            DecodedOpcode.stvis,
            DecodedOpcode.ldm,
            DecodedOpcode.ldvm,
            DecodedOpcode.stm,
            DecodedOpcode.stvm,
            DecodedOpcode.ldml,
            DecodedOpcode.ldvml,
            DecodedOpcode.stml,
            DecodedOpcode.stvml,
            DecodedOpcode.ldmq,
            DecodedOpcode.ldvmq,
            DecodedOpcode.stmq,
            DecodedOpcode.stvmq,
            => ArchitectureLevel.Extended,
            DecodedOpcode.cmpstr,
            DecodedOpcode.movqstr,
            DecodedOpcode.movstr,
            DecodedOpcode.inspacc,
            DecodedOpcode.ldphy,
            DecodedOpcode.fill,
            DecodedOpcode.condrec,
            DecodedOpcode.receive,
            DecodedOpcode.send,
            DecodedOpcode.sendserv,
            DecodedOpcode.resumprcs,
            DecodedOpcode.schedprcs,
            DecodedOpcode.saveprcs,
            DecodedOpcode.condwait,
            DecodedOpcode.wait,
            DecodedOpcode.signal,
            DecodedOpcode.ldtime,
            => ArchitectureLevel.Protected,
            DecodedOpcode.daddc,
            DecodedOpcode.dsubc,
            DecodedOpcode.dmovt,
            DecodedOpcode.cvtir,
            DecodedOpcode.cvtilr,
            DecodedOpcode.scalerl,
            DecodedOpcode.scaler,
            DecodedOpcode.atanr,
            DecodedOpcode.logepr,
            DecodedOpcode.logr,
            DecodedOpcode.remr,
            DecodedOpcode.cmpor,
            DecodedOpcode.cmpr,
            DecodedOpcode.sqrtr,
            DecodedOpcode.expr,
            DecodedOpcode.logbnr,
            DecodedOpcode.roundr,
            DecodedOpcode.sinr,
            DecodedOpcode.cosr,
            DecodedOpcode.tanr,
            DecodedOpcode.classr,
            DecodedOpcode.atanrl,
            DecodedOpcode.logeprl,
            DecodedOpcode.logrl,
            DecodedOpcode.remrl,
            DecodedOpcode.cmporl,
            DecodedOpcode.cmprl,
            DecodedOpcode.sqrtrl,
            DecodedOpcode.exprl,
            DecodedOpcode.logbnrl,
            DecodedOpcode.roundrl,
            DecodedOpcode.sinrl,
            DecodedOpcode.cosrl,
            DecodedOpcode.tanrl,
            DecodedOpcode.classrl,
            DecodedOpcode.cvtri,
            DecodedOpcode.cvtril,
            DecodedOpcode.cvtzri,
            DecodedOpcode.cvtzril,
            DecodedOpcode.movr,
            DecodedOpcode.movrl,
            DecodedOpcode.cpysre,
            DecodedOpcode.cpyrsre,
            DecodedOpcode.movre,
            DecodedOpcode.divr,
            DecodedOpcode.mulr,
            DecodedOpcode.subr,
            DecodedOpcode.addr,
            DecodedOpcode.divrl,
            DecodedOpcode.mulrl,
            DecodedOpcode.subrl,
            DecodedOpcode.addrl,
            => ArchitectureLevel.Numerics,
            else => ArchitectureLevel.Core,
        };
    }
    pub fn isFloatingPointInstruction(self: DecodedOpcode) bool {
        return switch (self) {
            DecodedOpcode.cvtir,
            DecodedOpcode.cvtilr,
            DecodedOpcode.scalerl,
            DecodedOpcode.scaler,
            DecodedOpcode.atanr,
            DecodedOpcode.logepr,
            DecodedOpcode.logr,
            DecodedOpcode.remr,
            DecodedOpcode.cmpor,
            DecodedOpcode.cmpr,
            DecodedOpcode.sqrtr,
            DecodedOpcode.expr,
            DecodedOpcode.logbnr,
            DecodedOpcode.roundr,
            DecodedOpcode.sinr,
            DecodedOpcode.cosr,
            DecodedOpcode.tanr,
            DecodedOpcode.classr,
            DecodedOpcode.atanrl,
            DecodedOpcode.logeprl,
            DecodedOpcode.logrl,
            DecodedOpcode.remrl,
            DecodedOpcode.cmporl,
            DecodedOpcode.cmprl,
            DecodedOpcode.sqrtrl,
            DecodedOpcode.exprl,
            DecodedOpcode.logbnrl,
            DecodedOpcode.roundrl,
            DecodedOpcode.sinrl,
            DecodedOpcode.cosrl,
            DecodedOpcode.tanrl,
            DecodedOpcode.classrl,
            DecodedOpcode.cvtri,
            DecodedOpcode.cvtril,
            DecodedOpcode.cvtzri,
            DecodedOpcode.cvtzril,
            DecodedOpcode.movr,
            DecodedOpcode.movrl,
            DecodedOpcode.cpysre,
            DecodedOpcode.cpyrsre,
            DecodedOpcode.movre,
            DecodedOpcode.divr,
            DecodedOpcode.mulr,
            DecodedOpcode.subr,
            DecodedOpcode.addr,
            DecodedOpcode.divrl,
            DecodedOpcode.mulrl,
            DecodedOpcode.subrl,
            DecodedOpcode.addrl,
            => true,
            else => false,
        };
    }
};

const CTRLInstruction = packed struct {
    displacement: i24,
    opcode: u8,

    pub fn getDisplacement(self: *const CTRLInstruction) i24 {
        const bits = @as(u24, @bitCast(self.displacement)) & 0xFFFFFC;
        return @as(i24, @bitCast(bits));
    }
    pub fn getOpcode(self: *const CTRLInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
};

const COBRInstruction = packed struct {
    displacement: i13,
    m1: bool,
    src2: Operand,
    src1: Operand,
    opcode: u8,

    pub fn getDisplacement(self: *const COBRInstruction) i13 {
        const bits = @as(u13, @bitCast(self.displacement)) & 0b1_1111_1111_1100;
        return @as(i13, @bitCast(bits));
    }
    pub fn treatSrc1AsLiteral(self: *const COBRInstruction) bool {
        return self.m1;
    }
    pub fn getOpcode(self: *const COBRInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
};

const REGInstruction = packed struct {
    src1: Operand,
    s1: bool = false,
    s2: bool = false,
    opcodeExt: u4,
    m1: bool,
    m2: bool,
    m3: bool,
    src2: Operand,
    srcDest: Operand,
    opcode: u8,
    pub fn treatSrc1AsLiteral(self: *const REGInstruction) bool {
        return self.m1;
    }
    pub fn treatSrc2AsLiteral(self: *const REGInstruction) bool {
        return self.m2;
    }
    pub fn getOpcode(self: *const REGInstruction) !DecodedOpcode {
        var major: u12 = @as(u12, self.opcode);
        major <<= 4;
        const minor: u12 = self.opcodeExt;
        //return @enumFromInt(major | minor);
        return meta.intToEnum(DecodedOpcode, major | minor) catch error.IllegalOpcode;
    }
};
const MEMAAddressComputationKind = enum(u1) {
    Offset = 0,
    @"(abase)+offset" = 1,
};
const MEMBAddressComputationKind = enum(u4) {
    @"(abase)" = 0b0100,
    @"(IP)+displacement+8" = 0b0101,
    @"(abase)+(index)*2^scale" = 0b0111,
    displacement = 0b1100,
    @"(abase)+displacement" = 0b1101,
    @"(index)*2^scale+displacement" = 0b1110,
    @"(abase)+(index)*2^scale+displacement" = 0b1111,
    pub fn usesOptionalDisplacement(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+displacement", MEMBAddressComputationKind.@"(index)*2^scale+displacement", MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement", MEMBAddressComputationKind.displacement, MEMBAddressComputationKind.@"(IP)+displacement+8" => true,
            else => false,
        };
    }
    pub fn usesScaleField(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale", MEMBAddressComputationKind.@"(index)*2^scale+displacement", MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement" => true,
            else => false,
        };
    }
};
const MEMAInstruction = packed struct {
    offset: u12,
    determinant: u1,
    mode: MEMAAddressComputationKind,
    abase: Operand,
    srcDest: Operand,
    opcode: u8,

    pub fn getOpcode(self: *const MEMAInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }

    pub fn getOffset(self: *const MEMAInstruction) u12 {
        return self.offset;
    }

    pub fn getComputationMode(self: *const MEMAInstruction) MEMAAddressComputationKind {
        return self.mode;
    }
};

const MEMBInstruction = packed struct {
    index: Operand,
    unused: u2,
    scale: u3,
    mode: MEMBAddressComputationKind,
    abase: Operand,
    srcDest: Operand,
    opcode: u8,

    pub fn getOpcode(self: *const MEMBInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
    pub fn usesOptionalDisplacement(self: *const MEMBInstruction) bool {
        return self.mode.usesOptionalDisplacement();
    }
};

const InstructionDeterminant = packed struct {
    unused0: u12,
    memDeterminant: u1,
    unused1: u11,
    opcode: u8,

    pub fn isCTRLInstruction(self: *const InstructionDeterminant) bool {
        return determineInstructionClass(self.opcode) == InstructionClass.CTRL;
    }
    pub fn isCOBRInstruction(self: *const InstructionDeterminant) bool {
        return determineInstructionClass(self.opcode) == InstructionClass.COBR;
    }
    pub fn isREGInstruction(self: *const InstructionDeterminant) bool {
        return determineInstructionClass(self.opcode) == InstructionClass.REG;
    }
    pub fn isMEMInstruction(self: *const InstructionDeterminant) bool {
        return determineInstructionClass(self.opcode) == InstructionClass.MEM;
    }
    pub fn isMEMAInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 0);
    }
    pub fn isMEMBInstruction(self: *const InstructionDeterminant) bool {
        return self.isMEMInstruction() and (self.memDeterminant == 1);
    }
};

const Instruction = union(enum) {
    ctrl: CTRLInstruction,
    cobr: COBRInstruction,
    reg: REGInstruction,
    mema: MEMAInstruction,
    memb: MEMBInstruction,

    pub fn getOpcode(self: *const Instruction) !DecodedOpcode {
        return switch (self.*) {
            .ctrl => |k| k.getOpcode(),
            .cobr => |k| k.getOpcode(),
            .reg => |k| k.getOpcode(),
            .mema => |k| k.getOpcode(),
            .memb => |k| k.getOpcode(),
        };
    }
    pub fn getSrcDest(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.srcDest,
            .mema => |k| k.srcDest,
            .memb => |k| k.srcDest,
            else => error.NoArgument,
        };
    }
    pub fn getSrc1(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src1,
            .mema => |k| k.src1,
            .memb => |k| k.src1,
            .cobr => |k| k.src1,
            else => error.NoArgument,
        };
    }
    pub fn getSrc2(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src2,
            .mema => |k| k.src2,
            .memb => |k| k.src2,
            .cobr => |k| k.src2,
            else => error.NoArgument,
        };
    }
};

fn decode(opcode: Ordinal) Instruction {
    const determinant: *const InstructionDeterminant = @ptrCast(&opcode);
    return switch (determineInstructionClass(determinant.opcode)) {
        InstructionClass.CTRL => Instruction{
            .ctrl = @as(*const CTRLInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.COBR => Instruction{
            .cobr = @as(*const COBRInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.REG => Instruction{
            .reg = @as(*const REGInstruction, @ptrCast(&opcode)).*,
        },
        InstructionClass.MEM => if (determinant.memDeterminant == 1) Instruction{
            .memb = @as(*const MEMBInstruction, @ptrCast(&opcode)).*,
        } else Instruction{
            .mema = @as(*const MEMAInstruction, @ptrCast(&opcode)).*,
        },
    };
}
const PFP: Operand = 0;
const SP: Operand = 1;
const RIP: Operand = 2;
const LinkRegister: Operand = 30; // g14
const FramePointer: Operand = 31;
fn getPFPAddress(value: Ordinal) Ordinal {
    return value & 0xFFFFFFF0;
}

const FaultRecord = packed struct {
    unused: u32,
    overrideFaultData: u96,
    faultData: u96,
    overrideSubType: u8,
    unused1: u8,
    overrideType: u8,
    overrideFlags: u8,
    processControls: u32,
    arithmeticControls: u32,
    faultSubtype: u8,
    unused2: u8,
    faultType: u8,
    faultFlags: u8,
    faultingInstructionAddress: u32,
};
const ProcessorControls = packed struct {
    unused0: u1 = 0,
    @"multiprocessor preempt": u1,
    state: u2,
    unused1: u1 = 0,
    @"nonpreempt limit": u5,
    @"addressing mode": u1,
    @"check dispatch port": u1,
    unused2: u5 = 0,
    @"interim priority": u5,
    unused3: u10 = 0,
    @"write external priority": u1,
};
const IACMessage = packed struct {
    field2: u16,
    field1: u8,
    messageType: u8,
    field3: u32,
    field4: u32,
    field5: u32,
};
const ArithmeticControls = packed struct {
    @"condition code": u3 = 0,
    @"arithmetic status": u4 = 0,
    unused0: u1 = 0,
    @"integer overflow flag": u1 = 0,
    unused1: u3 = 0,
    @"integer overflow mask": u1 = 0,
    unused2: u2 = 0,
    @"no imprecise faults": u1 = 0,
    @"floating overflow flag": u1 = 0,
    @"floating underflow flag": u1 = 0,
    @"floating invalid-op flag": u1 = 0,
    @"floating zero-divide flag": u1 = 0,
    @"floating inexact flag": u1 = 0,
    unused3: u3 = 0,
    @"floating overflow mask": u1 = 0,
    @"floating underflow mask": u1 = 0,
    @"floating invalid-op mask": u1 = 0,
    @"floating zero-divide mask": u1 = 0,
    @"floating inexact mask": u1 = 0,
    @"floating-point normalizing mode": u1 = 0,
    @"floating-point rounding control": u1 = 0,
};
const ProcessControls = packed struct {
    @"trace enable": u1 = 0,
    @"execution mode": u1 = 0,
    unused0: u4 = 0,
    @"time-slice reschedule": u1 = 0,
    @"time-slice": u1 = 0,
    timing: u1 = 0,
    @"resume": u1 = 0,
    @"trace-fault pending": u1 = 0,
    preempt: u1 = 0,
    refault: u1 = 0,
    state: u2 = 0,
    unused1: u1 = 0,
    priority: u5 = 0,
    @"internal state": u11 = 0,
};

const TraceControls = packed struct {
    unused0: u1 = 0,
    @"instruction trace mode": u1 = 0,
    @"branch trace mode": u1 = 0,
    @"call trace mode": u1 = 0,
    @"return trace mode": u1 = 0,
    @"prereturn trace mode": u1 = 0,
    @"supervisor trace mode": u1 = 0,
    @"breakpoint trace mode": u1 = 0,
    unused1: u9 = 0,
    @"instruction trace event": u1 = 0,
    @"branch trace event": u1 = 0,
    @"call trace event": u1 = 0,
    @"return trace event": u1 = 0,
    @"prereturn trace event": u1 = 0,
    @"supervisor trace event": u1 = 0,
    @"breakpoint trace event": u1 = 0,
    unused2: u8 = 0,
};

test "sizeof sanity check" {
    try expect(@sizeOf(ArithmeticControls) == 4);
    try expect(@sizeOf(ProcessControls) == 4);
    try expect(@sizeOf(TraceControls) == 4);
}

const Core = struct {
    globals: RegisterFrame,
    locals: [4]RegisterFrame,
    fpr: [4]ExtendedReal,
    ip: Ordinal = 0,
    currentLocalFrame: u2 = 0,
    advanceBy: u3 = 4,
    pc: ProcessControls,
    ac: ArithmeticControls,
    tc: TraceControls,
    fn getRegister(self: *Core, index: Operand) *u32 {
        if (index > 16) {
            return &self.globals[index and 0b1111];
        } else {
            return &self.locals[self.currentLocalFrame][index and 0b1111];
        }
    }
    fn getRegisterValue(self: *Core, index: Operand) u32 {
        return self.getRegister(index).*;
    }
    fn processInstruction(self: *Core, instruction: Instruction) !void {
        switch (instruction.getOpcode() catch |x| {
            return x;
        }) {
            DecodedOpcode.b => {
                self.advanceBy = 0;
                self.ip += instruction.ctrl.getDisplacement();
            },
            DecodedOpcode.bal => {
                self.advanceBy = 0;
                self.getRegister(LinkRegister).* = (self.ip + 4);
                self.ip += instruction.ctrl.getDisplacement();
            },
            DecodedOpcode.addo => {
                const dest = instruction.getSrcDest() catch |x| {
                    return x;
                };
                const src1 = instruction.getSrc1() catch |x| {
                    return x;
                };
                const src2 = instruction.getSrc2() catch |x| {
                    return x;
                };
                self.getRegister(dest).* = self.getRegisterValue(src2) + self.getRegisterValue(src1);
            },
            else => return error.Unimplemented,
        }
    }
};

pub fn main() void {
    std.debug.print("i960 Simulator\n", .{});
}

// test cases
test "instruction size tests" {
    try expect(@sizeOf(Ordinal) == 4);
    try expect(@sizeOf(CTRLInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(COBRInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(REGInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(MEMAInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(MEMBInstruction) == @sizeOf(Ordinal));
    try expect(@sizeOf(InstructionDeterminant) == @sizeOf(Ordinal));
}
test "simple reg test" {
    const x = REGInstruction{
        .src1 = 9,
        .opcodeExt = 0x2,
        .m1 = false,
        .m2 = true,
        .m3 = false,
        .src2 = 8,
        .srcDest = 7,
        .opcode = 0x58,
    };
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x582);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    } == DecodedOpcode.andnot);
    try expect(x.srcDest == 7);
    try expect(x.src2 == 8);
    try expect(x.src1 == 9);
}

test "simple ctrl test" {
    const x = CTRLInstruction{
        .displacement = 0xFDEC,
        .opcode = 0x8,
    };
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    } == DecodedOpcode.b);
    try expect(x.getDisplacement() == 0xFDEC);
}

test "simple ctrl test2" {
    var x: CTRLInstruction = .{
        .displacement = 0,
        .opcode = 0,
    };

    x.displacement = 0xFDEC;
    x.opcode = 0x08;
    try expect(@intFromEnum(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    }) == 0x08);
    try expect(x.getOpcode() catch |err| {
        try expect(err == error.IllegalOpcode);
        return;
    } == DecodedOpcode.b);
    try expect(x.getDisplacement() == 0xFDEC);
}

test "simple cobr test" {
    const x = COBRInstruction{
        .displacement = 0xFF,
        .m1 = false,
        .src1 = 4,
        .src2 = 5,
        .opcode = 0x32,
    };
    try expect(x.getDisplacement() == 0xFC);
    try expect(!x.m1);
    try expect(x.src1 == 4);
    try expect(x.src2 == 5);
    //const value: u32 = @bitCast(x);
    //std.debug.print("{x}\n", .{value});
}

test "MEMBFormat tests" {
    try expect(MEMBAddressComputationKind.displacement.usesOptionalDisplacement());
}

test "instruction decoder test 0" {
    const originalValue: u32 = 0x322140ff;

    try expect(switch (decode(originalValue)) {
        .cobr => true,
        else => false,
    });
}

test "instruction decoder test 1" {
    const originalValue: u32 = 0x0800_0000;

    try expect(switch (decode(originalValue)) {
        .ctrl => true,
        else => false,
    });
}

test "instruction decoder test 2" {
    const originalValue: u32 = 0x8000_0000;

    try expect(switch (decode(originalValue)) {
        .mema => true,
        else => false,
    });
}

test "Opcodes Sanity Checks" {
    try expect(@intFromEnum(DecodedOpcode.cmpibo) == 0x3f);
}

test "Opcodes Sanity Checks 2" {
    const originalValue: u32 = 0x3f00_0000;
    const decodedInstruction = decode(originalValue);
    const value = decodedInstruction.getOpcode() catch {
        try expect(false);
        return;
    };
    try expect(value == DecodedOpcode.cmpibo);
}

test "FaultRecord sanity check" {
    try expect(@sizeOf(FaultRecord) == 48);
}
