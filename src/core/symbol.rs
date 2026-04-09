use super::Key;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct Symbol(Key);

impl Symbol {
    pub fn new(chr: u8, index: u64) -> Self {
        let key = (u64::from(chr) << 56) | (index & 0x00ff_ffff_ffff_ffff);
        Self(key)
    }

    pub fn chr(self) -> u8 {
        (self.0 >> 56) as u8
    }

    pub fn index(self) -> u64 {
        self.0 & 0x00ff_ffff_ffff_ffff
    }

    pub fn key(self) -> Key {
        self.0
    }
}

impl From<Symbol> for Key {
    fn from(value: Symbol) -> Self {
        value.key()
    }
}

#[allow(non_snake_case)]
pub fn X(index: u64) -> Symbol {
    Symbol::new(b'x', index)
}

#[allow(non_snake_case)]
pub fn L(index: u64) -> Symbol {
    Symbol::new(b'l', index)
}

#[allow(non_snake_case)]
pub fn P(index: u64) -> Symbol {
    Symbol::new(b'p', index)
}
