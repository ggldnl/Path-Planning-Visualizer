class Vector {

    constructor(i, j) {
        this.i = i;
        this.j = j;
    }

    copy() {
        return new Vector(this.i, this.j);
    }

    magnitude() {
        return Math.sqrt(Math.pow(this.i, 2) + Math.pow(this.j, 2));
    }

    normalized() {
        return this.divide(this.magnitude());
    }

    add(vector) {
        const result = this.copy();
        result.i += vector.i;
        result.j += vector.j;
        return result;
    }

    subtract(vector) {
        const result = this.copy();
        result.i -= vector.i;
        result.j -= vector.j;
        return result;
    }

    divide(scalar) {
        const result = this.copy();
        result.i /= scalar;
        result.j /= scalar;
        return result;
    }

    multiply(scalar) {
        const result = this.copy();
        result.i *= scalar;
        result.j *= scalar;
        return result;
    }

    perpendicular() {
        return new Vector(-this.j, this.i);
    }

    transform(matrix) {
        const i = matrix.data[0][0] * this.i + matrix.data[0][1] * this.j;
        const j = matrix.data[1][0] * this.i + matrix.data[1][1] * this.j;
        return new Vector(i, j);
    }

    toString() {
        return `(${this.i}, ${this.j})`;
    }
}