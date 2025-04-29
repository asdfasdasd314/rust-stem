use crate::math_util::*;
use crate::float_precision::*;
use crate::hashable::*;
use std::collections::HashSet;

type Data = (Vector2f64, usize);

/**
Returns true if the first point should come before the second point,
and false if the second point should come before the first point

I understand this is what the key would be if I just used sort_by(), but I think there's optimizations I could do with a heap in the future
and I just wanted to implement a heap because it's cool, and now I have a much better understanding of them
 */
fn compare_order_of_two_points(comparison_axis: &ComparisonAxis, point1: &Data, point2: &Data) -> bool {
    let root_vec = comparison_axis.p1 - comparison_axis.p0;

    let vec1 = point1.0 - comparison_axis.p1;
    let vec2 = point2.0 - comparison_axis.p1;

    let cos_angle1 = calculate_cos_of_angle(&root_vec, &vec1);
    let cos_angle2 = calculate_cos_of_angle(&root_vec, &vec2);

    if broad_equal(cos_angle1, cos_angle2) {
        // If they are the same, then use the distance to the point
        let distance1 = vec1.length();
        let distance2 = vec2.length();

        return distance1 < distance2;
    }
    
    cos_angle1 > cos_angle2
}

/**
This is not a general heap, this a heap I specifically need to implement heap sort because I want to use Graham's Scan and stuff
This is a min heap where there are no duplicates and the sorting key is the angle from the `comparison_point`
 */
pub mod custom_heap {
    use super::*;

    pub fn get_min_point(heap: &[Data]) -> Data {
        heap[0]
    }

    fn get_parent_index(index: usize) -> Option<usize> {
        let parent_index: isize = (index as isize - 1) / 2;
        if parent_index < 0 {
            return None;
        }
        Some(parent_index as usize)
    }

    fn get_left_child_index(heap: &[Data], index: usize) -> Option<usize> {
        let left_child_index = 2 * index + 1;
        if left_child_index >= heap.len() {
            return None;
        }
        Some(left_child_index)
    }

    fn get_right_child_index(heap: &[Data], index: usize) -> Option<usize> {
        let right_child_index = 2 * index + 2;
        if right_child_index >= heap.len() {
            return None;
        }
        Some(right_child_index)
    }

    /**
    Moves an element up through the heap until the heap property is restored
    `actual_root` is the root point of the polygon
    `helper_node` is another point that defines the line for which all points will compare their angle with it to, this should be one -1 units from the actual root in the x direction
     */
    fn sift_up(
        comparison_axis: &ComparisonAxis,
        mut index: usize,
        heap: &mut [Data],
    ) {
        while index > 0 {
            let point = heap[index];
            let parent_index = get_parent_index(index).expect("This should not panic because we already checked that the index isn't the minimum index");

            let parent_point = heap[parent_index];

            let parent_first = compare_order_of_two_points(comparison_axis, &parent_point, &point);

            if parent_first {
                break;
            }

            heap[index] = parent_point;
            heap[parent_index] = point;
            index = parent_index;
        }
    }

    /**
    Moves an element down through the heap until the heap property is restored
    `actual_root` is the root point of the polygon
    `helper_node` is another point that defines the line for which all points will compare their angle with it to
     */
    fn sift_down(
        comparison_axis: &ComparisonAxis,
        mut index: usize,
        heap: &mut [Data],
    ) {
        while index < heap.len() {
            let left_index = get_left_child_index(heap, index);
            let max_index = match left_index {
                Some(left_index) => {
                    let mut swap = index;
                    if !compare_order_of_two_points(comparison_axis, &heap[index], &heap[left_index]) {
                        swap = left_index
                    }

                    let right_index = get_right_child_index(heap, index);
                    if let Some(right_index) = right_index {
                        if !compare_order_of_two_points(comparison_axis, &heap[swap], &heap[right_index]) {
                            swap = right_index;
                        }
                    }

                    swap
                }
                None => {
                    // Just give the index because we're at a leaf
                    index
                }
            };

            if max_index == index {
                break;
            }

            heap.swap(index, max_index);
            index = max_index;
        }
    }

    /**
    Turns an array into a heap
    `comparison_root` is the root point of the polygon
    `comparison_helper` is another point that defines the line for which all points will compare their angle with it to
     */
    pub fn heapify(
        comparison_axis: &ComparisonAxis,
        array: &mut Vec<Data>,
    ) {
        for i in (0..=array.len() / 2).rev() {
            // There is an issue with how the root is compared with other points,
            // and how the root might not be the bottom left point sometimes, should the root be the bottom leftmost point even if it doesn't contain the polygon?
            sift_down(comparison_axis, i, array);
        }
    }

    /**
    Pushes an element onto the heap
    `comparison_root` is the root point of the polygon
    `comparison_helper` is another point that defines the line for which all points will compare their angle with it to
     */
    pub fn heap_push(
        comparison_axis: &ComparisonAxis,
        new_element: Data,
        heap: &mut Vec<Data>,
    ) {
        // Push the element
        heap.push(new_element);

        // Restore the heap
        sift_up(comparison_axis, heap.len() - 1, heap);
    }

    /**
    Pops the min element from the heap and returns it, returns None if there are no elements to pop
    `actual_root` is the root point of the polygon
    `helper_point` is another point that defines the line for which all points will compare their angle with it to
     */
    pub fn heap_pop(
        comparison_axis: &ComparisonAxis,
        heap: &mut Vec<Data>,
    ) -> Option<Data> {
        if heap.is_empty() {
            return None;
        }

        // Pop at index 0 by placing the last element at index 0 and popping (O(1))
        let pop_element = heap[0];
        heap[0] = *heap.last().unwrap();
        heap.pop();

        // Restore the heap
        sift_down(comparison_axis, 0, heap);

        Some(pop_element)
    }

    /**
    Sorts the array using heapsort while removing duplicates
    `actual_root` is the root point of the polygon
    `comparison_point` is another point that defines the line for which all points will compare their angle with it to
     */
    pub fn heapsort(
        comparison_axis: &ComparisonAxis,
        array: &mut Vec<Data>,
    ) {
        heapify(comparison_axis, array);
        // We use Vec::new() and not Vec::with_capacity() because technically this will be O(1) space
        let mut used_points: HashSet<HashableVector2> = HashSet::new();
        let mut new_array: Vec<Data> = Vec::new();
        for _ in 0..array.len() {
            let top = heap_pop(comparison_axis, array).unwrap();
            let hashable_top: HashableVector2 = top.0.into();
            if !used_points.contains(&hashable_top) {
                used_points.insert(hashable_top);
                new_array.push(top);
            }

            // I 1000000% understand that this is slower, but for the purpose of the O(1) space, which honestly if I'm implementing heapsort, I'm more interested in the space
            // then this is better
            array.shrink_to_fit();
        }
        *array = new_array;
    }
}

pub fn is_sorted(comparison_axis: &ComparisonAxis, values: &[Data]) -> bool {
    for i in 0..values.len() - 1 {
        if !compare_order_of_two_points(comparison_axis, &values[i], &values[i + 1]) {
            return false;
        }
    }

    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use super::custom_heap::*;

    #[test]
    fn test_heapsort() {
        let mut points = vec![
            Vector2f64::new(0.0, 0.0),
            Vector2f64::new(1.0, 0.5),
            Vector2f64::new(1.0, 0.0),
            Vector2f64::new(1.0, 1.0),
            Vector2f64::new(0.0, 2.0),
            Vector2f64::new(1.0, 2.5),
            Vector2f64::new(2.0, 0.0),
            Vector2f64::new(1.0, 2.0),
            Vector2f64::new(-1.0, 1.0),
            Vector2f64::new(-2.0, 1.0),
            Vector2f64::new(-0.5, 1.0),
        ];

        let mut input: Vec<(Vector2f64, usize)> = points.iter().enumerate().map(|(index, point)| (*point, index)).collect();

        let mut comparison_axis = ComparisonAxis::new(&input);
        heapsort(&comparison_axis, &mut input);
        assert!(is_sorted(&comparison_axis, &input));

        points = vec![
            Vector2f64::new(1.0000001013443094, -4.7416475008521627e-5),
            Vector2f64::new(1.0000001013443094, 0.9999525835249915),
            Vector2f64::new(1.0134430934824675e-7, -4.7416475008521627e-5),
            Vector2f64::new(1.0134430934824675e-7, 0.9999525835249915),
        ];
        
        input = points.iter().enumerate().map(|(index, point)| (*point, index)).collect();

        comparison_axis = ComparisonAxis::new(&input);
        heapsort(&comparison_axis, &mut input);
   
        assert!(is_sorted(&comparison_axis, &input));
    }
}
