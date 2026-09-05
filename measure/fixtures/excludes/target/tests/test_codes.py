from hopper.store.codes import new_code


class FakeDB:
    def execute(self, *a):
        class R:
            def fetchone(self):
                return None
        return R()


def test_code_length():
    cfg = {"codes": {"length": 6, "alphabet": "abc"}}
    assert len(new_code(FakeDB(), cfg)) == 6
