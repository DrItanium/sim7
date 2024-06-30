const std = @import("std");
const meta = @import("std").meta;
const math = @import("std").math;
const expect = std.testing.expect;
const expect_eq = std.testing.expectEqual;
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
const MemoryPool = StorageFrame(ByteOrdinal, 4 * 1024 * 1024 * 1024);

// need to access byte by byte so we can reconstruct in a platform independent
// way
fn load(
    comptime T: type,
    pool: *MemoryPool,
    address: Address,
) T {
    // todo, at some point we need to take platform endianness into account!
    return switch (T) {
        ByteOrdinal, ByteInteger => @bitCast(pool[address]),
        ShortOrdinal, ShortInteger => {
            if ((address & 0b1) == 0) {
                const properView: *[]ShortOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 1]);
            } else {
                const a: ShortOrdinal = load(ByteOrdinal, pool, address);
                const b: ShortOrdinal = load(ByteOrdinal, pool, address +% 1);
                return @bitCast(a | (b << 8));
            }
        },
        Ordinal, Integer => {
            if ((address & 0b11) == 0) {
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 2]);
            } else {
                const a: Ordinal = load(ShortOrdinal, pool, address);
                const b: Ordinal = load(ShortOrdinal, pool, address +% 2);
                return @bitCast(a | (b << 16));
            }
        },
        LongOrdinal, LongInteger => {
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return @bitCast(properView.*[address >> 3]);
            } else {
                const a: LongOrdinal = load(Ordinal, pool, address);
                const b: LongOrdinal = load(Ordinal, pool, address +% 4);
                return @bitCast(a | (b << 32));
            }
        },
        TripleOrdinal => {
            if ((address & 0b1111) == 0) {
                // triple ordinals are strange as they are 96 bits in size.
                // However, the i960 treats them as aligned only on 16 byte
                // boundaries (same as quad ordinals) so we can use the
                // optimization only when aligned
                const properView: *[]TripleOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return properView.*[address >> 4];
            } else if ((address & 0b11) == 0) {
                // cool beans, the address is actually aligned to 32-bit
                // boundaries so we can actually do three separate optimized
                // ordinal loads
                const adjustedAddress = address >> 2;
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                const a: TripleOrdinal = properView.*[adjustedAddress +% 0];
                const b: TripleOrdinal = properView.*[adjustedAddress +% 1];
                const c: TripleOrdinal = properView.*[adjustedAddress +% 2];
                return a | (b << 32) | (c << 64);
            } else {
                // load three separate ordinals, it may turn out that some of
                // them are aligned actually!
                const a: TripleOrdinal = load(Ordinal, pool, address);
                const b: TripleOrdinal = load(Ordinal, pool, address +% 4);
                const c: TripleOrdinal = load(Ordinal, pool, address +% 8);
                return a | (b << 32) | (c << 64);
            }
        },
        QuadOrdinal => {
            if ((address & 0b1111) == 0) {
                // triple ordinals are strange as they are 96 bits in size.
                // However, the i960 treats them as aligned only on 16 byte
                // boundaries (same as quad ordinals) so we can use the
                // optimization only when aligned
                const properView: *[]QuadOrdinal = @ptrFromInt(@intFromPtr(&pool));
                return properView.*[address >> 4];
            } else if ((address & 0b111) == 0) {
                const adjustedAddress = address >> 3;
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                const a: QuadOrdinal = properView.*[adjustedAddress +% 0];
                const b: QuadOrdinal = properView.*[adjustedAddress +% 1];
                return a | (b << 64);
            } else if ((address & 0b11) == 0) {
                // cool beans, the address is actually aligned to 32-bit
                // boundaries so we can actually do three separate optimized
                // ordinal loads
                const adjustedAddress = address >> 2;
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                const a: QuadOrdinal = properView.*[adjustedAddress +% 0];
                const b: QuadOrdinal = properView.*[adjustedAddress +% 1];
                const c: QuadOrdinal = properView.*[adjustedAddress +% 2];
                const d: QuadOrdinal = properView.*[adjustedAddress +% 3];
                return a | (b << 32) | (c << 64) | (d << 96);
            } else {
                // load three separate ordinals, it may turn out that some of
                // them are aligned actually!
                const a: QuadOrdinal = load(Ordinal, pool, address);
                const b: QuadOrdinal = load(Ordinal, pool, address +% 4);
                const c: QuadOrdinal = load(Ordinal, pool, address +% 8);
                const d: QuadOrdinal = load(Ordinal, pool, address +% 12);
                return a | (b << 32) | (c << 64) | (d << 96);
            }
        },
        else => @compileError("Requested type not allowed!"),
    };
}
fn store(
    comptime T: type,
    pool: *MemoryPool,
    address: Address,
    value: T,
) void {
    switch (T) {
        ByteOrdinal, ByteInteger => pool[address] = @bitCast(value),
        ShortOrdinal, ShortInteger => {
            const view: ShortOrdinal = @bitCast(value);
            if ((address & 0b1) == 0) {
                const properView: *[]ShortOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 1] = view;
            } else {
                pool[address] = @truncate(view);
                pool[address +% 1] = @truncate(view >> 8);
            }
        },
        Ordinal, Integer => {
            const view: Ordinal = @bitCast(value);
            if ((address & 0b11) == 0) {
                const properView: *[]Ordinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 2] = view;
            } else {
                store(ShortOrdinal, pool, address, @truncate(view));
                store(ShortOrdinal, pool, address +% 2, @truncate(view >> 16));
            }
        },
        LongOrdinal, LongInteger => {
            const view: LongOrdinal = @bitCast(value);
            if ((address & 0b111) == 0) {
                const properView: *[]LongOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 3] = view;
            } else {
                store(Ordinal, pool, address, @truncate(view));
                store(Ordinal, pool, address +% 4, @truncate(view >> 32));
            }
        },
        TripleOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]TripleOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(Ordinal, pool, address, @truncate(value));
                store(Ordinal, pool, address +% 4, @truncate(value >> 32));
                store(Ordinal, pool, address +% 8, @truncate(value >> 64));
            }
        },
        QuadOrdinal => {
            if ((address & 0b1111) == 0) {
                const properView: *[]QuadOrdinal = @ptrFromInt(@intFromPtr(&pool));
                properView.*[address >> 4] = value;
            } else {
                store(LongOrdinal, pool, address, @truncate(value));
                store(LongOrdinal, pool, address +% 8, @truncate(value >> 64));
            }
        },
        else => @compileError("Requested type not supported!"),
    }
}

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

pub fn determineInstructionClass(opcode: u8) InstructionClass {
    return switch (opcode) {
        0x00...0x1F => InstructionClass.CTRL,
        0x20...0x3F => InstructionClass.COBR,
        0x40...0x7F => InstructionClass.REG,
        0x80...0xFF => InstructionClass.MEM,
    };
}
pub const DecodedOpcode = enum(u12) {
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
    movre = 0x6e1, // manuals state 0x6e9 but it is actually 0x6e1 (verified with an i960 and through the GNU binutils source code)
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
        return @truncate(switch (@intFromEnum(self)) {
            0x00...0xFF => @intFromEnum(self),
            else => @intFromEnum(self) >> 4,
        });
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
    fn getConditionCode(self: DecodedOpcode) u3 {
        return @truncate(self.getPrimaryOpcode() & 0b111);
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
    offset = 0,
    @"(abase)+offset" = 1,
};
const MEMBAddressComputationKind = enum(u4) {
    @"(abase)" = 0b0100,
    @"(ip)+displacement+8" = 0b0101,
    @"(abase)+(index)*2^scale" = 0b0111,
    displacement = 0b1100,
    @"(abase)+displacement" = 0b1101,
    @"(index)*2^scale+displacement" = 0b1110,
    @"(abase)+(index)*2^scale+displacement" = 0b1111,
    pub fn usesOptionalDisplacement(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+displacement",
            MEMBAddressComputationKind.@"(index)*2^scale+displacement",
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement",
            MEMBAddressComputationKind.displacement,
            MEMBAddressComputationKind.@"(ip)+displacement+8",
            => true,
            else => false,
        };
    }
    pub fn usesScaleField(self: MEMBAddressComputationKind) bool {
        return switch (self) {
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale",
            MEMBAddressComputationKind.@"(index)*2^scale+displacement",
            MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement",
            => true,
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
    index: Operand = 0,
    unused: u2 = 0,
    scale: u3 = 0,
    mode: u4 = 0,
    abase: Operand = 0,
    srcDest: Operand = 0,
    opcode: u8 = 0,

    pub fn getOpcode(self: *const MEMBInstruction) !DecodedOpcode {
        return meta.intToEnum(DecodedOpcode, self.opcode) catch error.IllegalOpcode;
    }
    pub fn usesOptionalDisplacement(self: *const MEMBInstruction) bool {
        return self.mode.usesOptionalDisplacement();
    }
    pub fn getComputationMode(self: *const MEMBInstruction) !MEMBAddressComputationKind {
        return meta.intToEnum(MEMBAddressComputationKind, self.mode) catch error.IllegalOperand;
    }
};

test "Bad MEMB Instruction" {
    const tmp = MEMBInstruction{
        .mode = 0b0110,
    };
    try expect(tmp.getComputationMode() == error.IllegalOperand);
}

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
            //.mema => |k| k.src1,
            //.memb => |k| k.src1,
            .cobr => |k| k.src1,
            else => error.NoArgument,
        };
    }
    pub fn getSrc2(self: *const Instruction) !Operand {
        return switch (self.*) {
            .reg => |k| k.src2,
            //.mema => |k| k.src2,
            //.memb => |k| k.src2,
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
const FP = FramePointer;
fn getPFPAddress(value: Ordinal) Ordinal {
    return value & 0xFFFFFFF0;
}
const GenericSegmentDescriptor = packed struct {
    reserved: u64 = 0,
    @"base address": Ordinal = 0,
    valid: u1 = 0,
    @"paging method": u2 = 0,
    @"access status": u5 = 0,
    unused0: u10 = 0,
    size: u6 = 0,
    unused1: u4 = 0,
    @"segment type": u4 = 0,

    pub fn isValid(self: *const GenericSegmentDescriptor) bool {
        return self.valid;
    }
};
const FaultProcedureEntry = packed struct {
    @"procedure address": u32 = 0,
    @"segment selector": u32 = 0,
    pub fn getProcedureIndex(self: *const FaultProcedureEntry) Address {
        return self.@"procedure address" & 0xFFFF_FFFC;
    }
    pub fn wholeValue(self: *const FaultProcedureEntry) u64 {
        return @as(*u64, @ptrCast(self)).*;
    }
};

const FaultTable = packed struct {
    @"override entry": FaultProcedureEntry,
    @"trace fault entry": FaultProcedureEntry,
    @"operation fault entry": FaultProcedureEntry,
    @"arithmetic fault entry": FaultProcedureEntry,
    @"floating-point fault entry": FaultProcedureEntry,
    @"constraint fault entry": FaultProcedureEntry,
    @"virtual-memory fault entry": FaultProcedureEntry,
    @"protection fault entry": FaultProcedureEntry,
    @"machine fault entry": FaultProcedureEntry,
    @"structure fault entry": FaultProcedureEntry,
    @"type fault entry": FaultProcedureEntry,
    reserved0: u64 = 0,
    @"process fault entry": FaultProcedureEntry,
    @"descriptor fault entry": FaultProcedureEntry,
    @"event fault entry": FaultProcedureEntry,
    reserved1: u1088 = 0,
    pub fn wholeValue(self: *const FaultProcedureEntry) u2048 {
        return @as(*u2048, @ptrCast(self)).*;
    }
};
const FaultRecord = packed struct {
    unused: u32 = 0,
    @"override fault data": u96 = 0,
    @"fault data": u96 = 0,
    @"override subtype": u8 = 0,
    unused1: u8 = 0,
    @"override type": u8 = 0,
    @"override flags": u8 = 0,
    @"process controls": u32,
    @"arithmetic controls": u32,
    @"fault subtype": u8 = 0,
    unused2: u8 = 0,
    @"fault type": u8 = 0,
    @"fault flags": u8 = 0,
    @"address of faulting instruction": u32,
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

    pub fn toWholeValue(self: *const ProcessorControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
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
    @"floating-point rounding control": u2 = 0,

    pub fn toWholeValue(self: *const ArithmeticControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
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
    pub fn toWholeValue(self: *const ProcessControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
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

    pub fn toWholeValue(self: *const TraceControls) Ordinal {
        return @as(*Ordinal, @ptrCast(self)).*;
    }
};
// there really isn't a point in keeping the instruction processing within the
// Core structure
const Core = struct {
    memory: *MemoryPool = undefined,
    globals: RegisterFrame = RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    locals: [4]RegisterFrame = [_]RegisterFrame{
        RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
        RegisterFrame{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
    },
    currentLocalFrame: u2 = 0,
    fpr: [4]ExtendedReal = [_]ExtendedReal{ 0.0, 0.0, 0.0, 0.0 },
    ip: Ordinal = 0,
    advanceBy: u4 = 4,
    pc: ProcessControls = @bitCast(@as(u32, 0)),
    ac: ArithmeticControls = @bitCast(@as(u32, 0)),
    tc: TraceControls = @bitCast(@as(u32, 0)),
    continueExecuting: bool = true,
    fn newCycle(self: *Core) void {
        self.advanceBy = 4;
    }
    fn nextInstruction(self: *Core) void {
        if (self.advanceBy > 0) {
            self.ip +%= self.advanceBy;
            self.advanceBy = 0;
        }
    }
    fn getRegister(self: *Core, index: Operand) *Ordinal {
        if (index > 16) {
            return &self.globals[index & 0b1111];
        } else {
            return &self.locals[self.currentLocalFrame][index & 0b1111];
        }
    }

    fn getFloatingPointRegister(self: *Core, index: Operand) !*ExtendedReal {
        return switch (index) {
            0b00000...0b00011 => |val| &self.fpr[val],
            else => error.IllegalOperand,
        };
    }
    fn setRegisterValue(self: *Core, index: Operand, value: Ordinal) void {
        self.getRegister(index).* = value;
    }
    fn getRegisterValue(self: *Core, index: Operand) Ordinal {
        return self.getRegister(index).*;
    }
    fn getTripleRegisterValue(self: *Core, index: Operand) !TripleOrdinal {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcodeFault;
        }
        const a: TripleOrdinal = self.getRegisterValue(index);
        const b: TripleOrdinal = self.getRegisterValue(index + 1);
        const c: TripleOrdinal = self.getRegisterValue(index + 2);
        return a | math.shl(TripleOrdinal, b, 32) | math.shl(TripleOrdinal, c, 64);
    }
    fn getQuadRegisterValue(self: *Core, index: Operand) !QuadOrdinal {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcodeFault;
        }
        const a: QuadOrdinal = self.getRegisterValue(index);
        const b: QuadOrdinal = self.getRegisterValue(index + 1);
        const c: QuadOrdinal = self.getRegisterValue(index + 2);
        const d: QuadOrdinal = self.getRegisterValue(index + 3);
        return a | math.shl(QuadOrdinal, b, 32) | math.shl(QuadOrdinal, c, 64) | math.shl(QuadOrdinal, d, 96);
    }
    fn getLongRegisterValue(self: *Core, index: Operand) !LongOrdinal {
        if ((index & 0b1) != 0) {
            return error.InvalidOpcodeFault;
        }
        const lower: LongOrdinal = self.getRegisterValue(index);
        const upper: LongOrdinal = self.getRegisterValue(index + 1);
        return lower | math.shl(LongOrdinal, upper, 32);
    }
    fn setLongRegisterValue(self: *Core, index: Operand, value: LongOrdinal) !void {
        if ((index & 0b1) != 0) {
            return error.InvalidOpcodeFault;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(LongOrdinal, value, 32)));
    }
    fn setTripleRegisterValue(self: *Core, index: Operand, value: TripleOrdinal) !void {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcodeFault;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(TripleOrdinal, value, 32)));
        self.setRegisterValue(index + 2, @truncate(math.shr(TripleOrdinal, value, 64)));
    }
    fn setQuadRegisterValue(self: *Core, index: Operand, value: QuadOrdinal) !void {
        if ((index & 0b11) != 0) {
            return error.InvalidOpcodeFault;
        }
        self.setRegisterValue(index, @truncate(value));
        self.setRegisterValue(index + 1, @truncate(math.shr(QuadOrdinal, value, 32)));
        self.setRegisterValue(index + 2, @truncate(math.shr(QuadOrdinal, value, 64)));
        self.setRegisterValue(index + 3, @truncate(math.shr(QuadOrdinal, value, 96)));
    }
    fn relativeBranch(
        self: *Core,
        comptime T: type,
        displacement: T,
    ) void {
        self.advanceBy = 0;
        var theip: Integer = @bitCast(self.ip);
        theip += displacement;
        self.ip = @bitCast(theip);
    }
    fn saveReturnAddress(self: *Core, dest: Operand) void {
        self.setRegisterValue(dest, self.ip + self.advanceBy);
    }
    fn newLocalRegisterFrame(self: *Core) void {
        // if registerSetAvailable
        // then allocate as new frame
        // else save a register set in memory at its FP; allocate as new frame
        _ = self;
        // @todo implement
    }
    fn moveRegisterValue(self: *Core, dest: Operand, src: Operand) void {
        self.setRegisterValue(dest, self.getRegisterValue(src));
    }
    fn loadFromMemory(
        self: *Core,
        comptime T: type,
        addr: Address,
    ) T {
        return switch (addr) {
            0xFE00_0000...0xFEFF_FFFF => 0,
            else => load(T, self.memory, addr),
        };
    }
    fn storeToMemory(
        self: *Core,
        comptime T: type,
        addr: Address,
        value: T,
    ) void {
        switch (addr) {
            // io space detection
            0xFE00_0000...0xFEFF_FFFF => {},
            else => store(T, self.memory, addr, value),
        }
    }
    fn computeEffectiveAddress(
        self: *Core,
        instruction: Instruction,
    ) !Address {
        return switch (instruction) {
            .mema => |inst| {
                return switch (inst.getComputationMode()) {
                    MEMAAddressComputationKind.offset => inst.getOffset(),
                    MEMAAddressComputationKind.@"(abase)+offset" => @bitCast(self.getRegisterValue(inst.abase) + inst.getOffset()),
                };
            },
            .memb => |inst| {
                return switch (try inst.getComputationMode()) {
                    MEMBAddressComputationKind.@"(abase)" => @bitCast(self.getRegisterValue(inst.abase)),
                    MEMBAddressComputationKind.@"(ip)+displacement+8" => {
                        // okay so this is the first instruction that needs to
                        // read the next word
                        const nextWord: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const theIP: Integer = @bitCast(self.ip);
                        const outcome: Integer = theIP + nextWord + 8;
                        self.advanceBy = 8;
                        return @bitCast(outcome);
                    },
                    MEMBAddressComputationKind.@"(abase)+(index)*2^scale" => @bitCast(self.getRegisterValue(inst.abase) +% (self.getRegisterValue(inst.index) << inst.scale)),
                    MEMBAddressComputationKind.displacement => {
                        self.advanceBy = 8;
                        const outcome: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        return @bitCast(outcome);
                    },
                    MEMBAddressComputationKind.@"(abase)+displacement" => {
                        self.advanceBy = 8;
                        const outcome: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const abase: Integer = @bitCast(self.getRegisterValue(inst.abase));
                        return @bitCast(outcome +% abase);
                    },
                    MEMBAddressComputationKind.@"(index)*2^scale+displacement" => {
                        self.advanceBy = 8;
                        const displacement: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const idx: Integer = @bitCast(self.getRegisterValue(inst.index));
                        const scale = inst.scale;
                        return @bitCast((idx << scale) + displacement);
                    },
                    MEMBAddressComputationKind.@"(abase)+(index)*2^scale+displacement" => {
                        self.advanceBy = 8;
                        const displacement: Integer = self.loadFromMemory(Integer, self.ip + 4);
                        const idx: Integer = @bitCast(self.getRegisterValue(inst.index));
                        const scale = inst.scale;
                        const abase: Integer = @bitCast(self.getRegisterValue(inst.abase));
                        return @bitCast(abase + (idx << scale) + displacement);
                    },
                };
            },
            else => error.NotMemInstruction,
        };
    }
};
fn computeBitpos(position: u5) Ordinal {
    return @as(Ordinal, 1) << position;
}
fn processInstruction(core: *Core, instruction: Instruction) !void {
    switch (try instruction.getOpcode()) {
        DecodedOpcode.b => core.relativeBranch(i24, instruction.ctrl.getDisplacement()),
        DecodedOpcode.bx => core.relativeBranch(i32, @bitCast(try core.computeEffectiveAddress(instruction))),
        DecodedOpcode.call,
        DecodedOpcode.callx,
        => |op| {
            // wait for any uncompleted instructions to finish
            const temp = (core.getRegisterValue(SP) + 63) & (~@as(Ordinal, 63)); // round to next boundary
            // save the return address to RIP in the _current_ frame
            core.saveReturnAddress(RIP);
            core.newLocalRegisterFrame();
            // at this point, all local register references are to the new
            // frame
            switch (op) {
                DecodedOpcode.call => core.relativeBranch(i24, instruction.ctrl.getDisplacement()),
                DecodedOpcode.callx => core.ip = try core.computeEffectiveAddress(instruction),
                else => unreachable,
            }
            core.moveRegisterValue(PFP, FP);
            core.setRegisterValue(FP, temp);
            core.setRegisterValue(SP, temp + 64);
        },
        DecodedOpcode.bal => {
            core.setRegisterValue(LinkRegister, core.ip + core.advanceBy);
            core.relativeBranch(i24, instruction.ctrl.getDisplacement());
        },
        DecodedOpcode.balx => {
            // compute the effective address first since the link register
            // could be included in the computation
            const destination = try core.computeEffectiveAddress(instruction);
            core.setRegisterValue(LinkRegister, core.ip + core.advanceBy);
            core.relativeBranch(i32, @bitCast(destination));
        },
        DecodedOpcode.bno => {
            if (core.ac.@"condition code" == 0b000) {
                core.relativeBranch(i24, instruction.ctrl.getDisplacement());
            }
        },
        DecodedOpcode.bg,
        DecodedOpcode.be,
        DecodedOpcode.bge,
        DecodedOpcode.bl,
        DecodedOpcode.bne,
        DecodedOpcode.ble,
        DecodedOpcode.bo,
        => |opcode| {
            if ((opcode.getConditionCode() & core.ac.@"condition code") != 0) {
                core.relativeBranch(i24, instruction.ctrl.getDisplacement());
            }
        },

        DecodedOpcode.faultno => {
            if (core.ac.@"condition code" == 0) {
                return error.ConstraintRangeFault;
            }
        },
        DecodedOpcode.faultg,
        DecodedOpcode.faulte,
        DecodedOpcode.faultge,
        DecodedOpcode.faultl,
        DecodedOpcode.faultne,
        DecodedOpcode.faultle,
        DecodedOpcode.faulto,
        => |opcode| {
            if ((opcode.getConditionCode() & core.ac.@"condition code") != 0) {
                return error.ConstraintRangeFault;
            }
        },
        DecodedOpcode.testno => core.setRegisterValue(instruction.getSrc1() catch unreachable, if (core.ac.@"condition code" == 0b000) 1 else 0),
        DecodedOpcode.teste,
        DecodedOpcode.testne,
        DecodedOpcode.testl,
        DecodedOpcode.testle,
        DecodedOpcode.testg,
        DecodedOpcode.testge,
        DecodedOpcode.testo,
        => |opcode| core.setRegisterValue(instruction.getSrc1() catch unreachable, if ((core.ac.@"condition code" & opcode.getConditionCode()) != 0) 1 else 0),
        DecodedOpcode.bbc => {
            // branch if bit clear
            const displacement = instruction.cobr.getDisplacement();
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.cobr.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src = core.getRegisterValue(src2Index);
            if ((src & bitpos) == 0) {
                core.ac.@"condition code" = 0b010;
                core.relativeBranch(@TypeOf(displacement), displacement);
                // resume execution at the new ip
            } else {
                core.ac.@"condition code" = 0b000;
            }
        },

        DecodedOpcode.bbs => {
            // branch if bit set
            const displacement = instruction.cobr.getDisplacement();
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.cobr.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src = core.getRegisterValue(src2Index);
            if ((src & bitpos) != 0) {
                core.ac.@"condition code" = 0b010;
                core.relativeBranch(@TypeOf(displacement), displacement);
                // resume execution at the new ip
            } else {
                core.ac.@"condition code" = 0b000;
            }
        },
        DecodedOpcode.emul => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: LongOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: LongOrdinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            try core.setLongRegisterValue(srcDestIndex, src2 *% src1);
        },
        DecodedOpcode.rotate => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const len: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, math.rotl(Ordinal, src, len & 0b11111));
        },
        DecodedOpcode.scanbyte => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if (((src1 & 0x000000FF) == (src2 & 0x000000FF)) or
                ((src1 & 0x0000FF00) == (src2 & 0x0000FF00)) or
                ((src1 & 0x00FF0000) == (src2 & 0x00FF0000)) or
                ((src1 & 0xFF000000) == (src2 & 0xFF000000))) 0b010 else 0b000;
        },
        DecodedOpcode.setbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src | bitpos);
        },
        DecodedOpcode.clrbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src & (~bitpos));
        },
        DecodedOpcode.notbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, src ^ bitpos);
        },
        DecodedOpcode.chkbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if ((src & bitpos) == 0) 0b000 else 0b010;
        },
        DecodedOpcode.cmpo => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.ac.@"condition code" = if (src1 < src2) 0b100 else if (src1 == src2) 0b010 else 0b001;
        },
        DecodedOpcode.cmpi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            core.ac.@"condition code" = if (src1 < src2) 0b100 else if (src1 == src2) 0b010 else 0b001;
        },
        DecodedOpcode.@"and",
        DecodedOpcode.@"or",
        DecodedOpcode.andnot,
        DecodedOpcode.notand,
        DecodedOpcode.ornot,
        DecodedOpcode.notor,
        DecodedOpcode.nand,
        DecodedOpcode.nor,
        DecodedOpcode.xor,
        DecodedOpcode.xnor,
        DecodedOpcode.mulo,
        DecodedOpcode.shro,
        DecodedOpcode.shlo,
        DecodedOpcode.not,
        DecodedOpcode.addo,
        DecodedOpcode.mov,
        => |operation| {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src2: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            // aliases
            const len = src1;
            const src = src2;
            core.setRegisterValue(srcDestIndex, switch (comptime operation) {
                DecodedOpcode.@"and" => src2 & src1,
                DecodedOpcode.andnot => src2 & (~src1),
                DecodedOpcode.notand => (~src2) & src1,
                DecodedOpcode.@"or" => src2 | src1,
                DecodedOpcode.ornot => src2 | (~src1),
                DecodedOpcode.notor => (~src2) | src1,
                DecodedOpcode.nand => (~src2) | (~src1),
                DecodedOpcode.nor => (~src2) & (~src1),
                DecodedOpcode.xor => src2 ^ src1,
                DecodedOpcode.xnor => ~(src2 ^ src1),
                DecodedOpcode.mulo => src2 *% src1,
                DecodedOpcode.not => ~src1,
                DecodedOpcode.shlo => if (len < 32) src << @truncate(len) else 0,
                DecodedOpcode.shro => if (len < 32) src >> @truncate(len) else 0,
                DecodedOpcode.addo => src2 +% src1,
                DecodedOpcode.mov => src1,
                else => unreachable,
            });
        },
        DecodedOpcode.muli => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src1Index else core.getRegisterValue(src2Index));
            // support detecting integer overflow here
            core.setRegisterValue(srcDestIndex, @bitCast(try math.mul(Integer, src2, src1)));
        },
        DecodedOpcode.modify => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const mask: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            const srcDest = core.getRegisterValue(srcDestIndex);
            core.setRegisterValue(srcDestIndex, modify(mask, src, srcDest));
        },
        DecodedOpcode.alterbit => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const bitpos: Ordinal = computeBitpos(@truncate((if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index)) & 0b11111)); // bitpos mod 32
            const src: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, alterbit(src, bitpos, (core.ac.@"condition code" & 0b010) == 0));
        },
        DecodedOpcode.syncf => {
            // do nothing
        },
        DecodedOpcode.movl => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: LongOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getLongRegisterValue(src1Index);
            try core.setLongRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.movt => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: TripleOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getTripleRegisterValue(src1Index);
            try core.setTripleRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.movq => {
            // be as simple as possible
            const src1Index = instruction.getSrc1() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const srcValue: QuadOrdinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else try core.getQuadRegisterValue(src1Index);
            try core.setQuadRegisterValue(srcDestIndex, srcValue);
        },
        DecodedOpcode.addi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const src1: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const src2: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src1Index else core.getRegisterValue(src2Index));
            // support detecting integer overflow here
            core.setRegisterValue(srcDestIndex, @bitCast(try math.add(Integer, src2, src1)));
        },
        DecodedOpcode.divo => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Ordinal = if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index);
            const numerator: Ordinal = if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index);
            core.setRegisterValue(srcDestIndex, math.divExact(Ordinal, numerator, denominator) catch |err| {
                if (err == error.DivisionByZero) {
                    core.nextInstruction();
                }
                return err;
            });
        },
        DecodedOpcode.divi => {
            const src1Index = instruction.getSrc1() catch unreachable;
            const src2Index = instruction.getSrc2() catch unreachable;
            const srcDestIndex = instruction.getSrcDest() catch unreachable;
            const denominator: Integer = @bitCast(if (instruction.reg.treatSrc1AsLiteral()) src1Index else core.getRegisterValue(src1Index));
            const numerator: Integer = @bitCast(if (instruction.reg.treatSrc2AsLiteral()) src2Index else core.getRegisterValue(src2Index));
            core.setRegisterValue(srcDestIndex, @bitCast(math.divExact(Integer, numerator, denominator) catch |err| {
                if (err == error.DivisionByZero) {
                    core.nextInstruction();
                }
                return err;
            }));
        },
        else => return error.Unimplemented,
    }
}
fn modify(mask: Ordinal, src: Ordinal, srcDest: Ordinal) Ordinal {
    return (src & mask) | (srcDest & (~mask));
}
fn alterbit(src: Ordinal, bitpos: Ordinal, clearBit: bool) Ordinal {
    return if (clearBit) (src & (~bitpos)) else (src | bitpos);
}
test "alterbit logic" {
    try expect_eq(alterbit(0, (1 << 24), false), 0x0100_0000);
}
test "modify logic" {
    try expect_eq(modify(0xFF, 0xFFFF, 0), 0xFF);
}
test "overflow test" {
    const a: i32 = 0x7FFF_FFFF;
    const b: i32 = 1;
    try expect((a +% b) < 0);
    const c: u32 = 0xFFFF_FFFF;
    const d: u32 = 1;
    const e = math.add(u32, c, d) catch 33;
    try expect(e == 33);
}
test "allocation test" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u8, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind);
    }
    for (buffer, 0..) |val, ind| {
        try expect_eq(val, ind);
    }
}
test "allocation test2" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u32, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + 0x03020100);
    }
    for (buffer, 0..) |val, ind| {
        try expect_eq(val, (ind + 0x03020100));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [4]u8 = @ptrCast(&buffer[0]);
    try expect_eq(buffer2[0], 0x00);
    try expect_eq(buffer2[1], 0x01);
    try expect_eq(buffer2[2], 0x02);
    try expect_eq(buffer2[3], 0x03);
}

pub fn main() !void {
    std.debug.print("i960 Simulator\n", .{});
    // allocate all of the memory at once
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.create(MemoryPool);
    defer allocator.destroy(buffer);
    var core = Core{
        .memory = buffer,
    };
    for (buffer) |*cell| {
        cell.* = 0;
    }
    while (core.continueExecuting) {
        core.newCycle();
        const result = decode(core.loadFromMemory(Ordinal, core.ip));
        processInstruction(&core, result) catch |x| {
            return x;
        };

        core.nextInstruction();
    }
}

// test cases

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

test "sizeof sanity check" {
    try expect(@sizeOf(ArithmeticControls) == 4);
    try expect(@sizeOf(ProcessControls) == 4);
    try expect(@sizeOf(TraceControls) == 4);
    try expect_eq(@sizeOf(FaultTable), 256);
}

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
    const value: u32 = @bitCast(x);
    std.debug.print("{x}\n", .{value});
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

test "allocation test3" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(u64, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + 0x0706050403020100);
    }
    for (buffer, 0..) |val, ind| {
        try expect_eq(val, (ind + 0x0706050403020100));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [@sizeOf(u64)]u8 = @ptrCast(&buffer[0]);
    for (buffer2, 0..) |value, idx| {
        try expect_eq(value, idx);
    }
    // view as two ordinals instead
    const buffer3: *const [@sizeOf(u64) / @sizeOf(Ordinal)]Ordinal = @ptrCast(&buffer[0]);
    try expect(buffer3[0] == 0x03020100 or buffer3[0] == 0x07060504);
    try expect(buffer3[1] == 0x03020100 or buffer3[1] == 0x07060504);
}

test "allocation test4" {
    const AllocationType = u128;
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.alloc(AllocationType, 4);
    defer allocator.free(buffer);
    try expect(buffer.len == 4);
    const combinatorial: AllocationType = 0x0f0e0d0c_0b0a0908_07060504_03020100;
    for (buffer, 0..) |*val, ind| {
        val.* = @truncate(ind + combinatorial);
    }
    for (buffer, 0..) |val, ind| {
        try expect_eq(val, (ind + combinatorial));
    }
    // okay, so now we need to see if we can view the various components
    //
    // it is trivial to go from larger to smaller!
    const buffer2: *const [@sizeOf(AllocationType)]u8 = @ptrCast(&buffer[0]);
    for (buffer2, 0..) |value, idx| {
        try expect_eq(value, idx);
    }
}

test "Structure Size Check 2" {
    try expect_eq(@sizeOf(MemoryPool), (4 * 1024 * 1024 * 1024));
}
test "memory pool load/store test" {
    const allocator = std.heap.page_allocator;
    const buffer = try allocator.create(MemoryPool);
    defer allocator.destroy(buffer);
    buffer[0] = 0xED;
    buffer[1] = 0xFD;
    buffer[2] = 0xFF;
    buffer[3] = 0xFF;
    buffer[4] = 0xab;
    buffer[5] = 0xcd;
    buffer[6] = 0xef;
    buffer[7] = 0x01;
    buffer[8] = 0x23;
    buffer[9] = 0x45;
    buffer[10] = 0x67;
    buffer[11] = 0x89;
    store(Ordinal, buffer, 12, 0x44332211);
    store(LongOrdinal, buffer, 16, 0xccbbaa99_88776655);
    store(ShortOrdinal, buffer, 24, 0xeedd);
    store(ByteOrdinal, buffer, 26, 0xff);
    try expect_eq(load(ByteOrdinal, buffer, 16), 0x55);
    try expect_eq(load(ByteOrdinal, buffer, 0), 0xED);
    try expect_eq(load(ByteOrdinal, buffer, 1), 0xFD);
    try expect_eq(load(ByteInteger, buffer, 2), -1);
    try expect_eq(load(ShortOrdinal, buffer, 0), 0xFDED);
    try expect_eq(load(ShortInteger, buffer, 2), -1);
    try expect_eq(load(ShortOrdinal, buffer, 2), 0xFFFF);
    try expect_eq(load(Ordinal, buffer, 0), 0xFFFFFDED);
    try expect_eq(load(TripleOrdinal, buffer, 0), 0x89674523_01efcdab_ffffFDED);
    try expect_eq(load(TripleOrdinal, buffer, 1), 0x1189674523_01efcdab_ffffFD);
    try expect_eq(load(TripleOrdinal, buffer, 2), 0x221189674523_01efcdab_ffff);
    try expect_eq(load(QuadOrdinal, buffer, 0), 0x44332211_89674523_01efcdab_ffffFDED);
    try expect_eq(load(QuadOrdinal, buffer, 1), 0x55443322_11896745_2301efcd_abffffFD);
    try expect_eq(load(ShortOrdinal, buffer, 24), 0xeedd);
}
