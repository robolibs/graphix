use super::Id;

#[derive(Debug, Clone)]
pub struct Store<T> {
    data: Vec<Option<T>>,
    free_list: Vec<u32>,
}

impl<T> Store<T> {
    pub fn new() -> Self {
        Self {
            data: Vec::new(),
            free_list: Vec::new(),
        }
    }

    pub fn add(&mut self, value: T) -> Id<T> {
        if let Some(slot) = self.free_list.pop() {
            self.data[slot as usize] = Some(value);
            Id::new(slot)
        } else {
            let slot = self.data.len() as u32;
            self.data.push(Some(value));
            Id::new(slot)
        }
    }

    pub fn remove(&mut self, id: Id<T>) -> Option<T> {
        let slot = id.value() as usize;
        let value = self.data.get_mut(slot)?.take();
        if value.is_some() {
            self.free_list.push(id.value());
        }
        value
    }

    pub fn get(&self, id: Id<T>) -> Option<&T> {
        self.data.get(id.value() as usize)?.as_ref()
    }

    pub fn get_mut(&mut self, id: Id<T>) -> Option<&mut T> {
        self.data.get_mut(id.value() as usize)?.as_mut()
    }

    pub fn contains(&self, id: Id<T>) -> bool {
        self.get(id).is_some()
    }

    pub fn len(&self) -> usize {
        self.data.iter().filter(|entry| entry.is_some()).count()
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    pub fn ids(&self) -> impl Iterator<Item = Id<T>> + '_ {
        self.data
            .iter()
            .enumerate()
            .filter(|(_, entry)| entry.is_some())
            .map(|(index, _)| Id::new(index as u32))
    }
}

impl<T> Default for Store<T> {
    fn default() -> Self {
        Self::new()
    }
}
