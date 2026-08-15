#include "ecs/world.hpp"
#include "test.hpp"

#include <algorithm>
#include <random>
#include <unordered_map>
#include <unordered_set>

using namespace skein;

namespace {
struct Pos {
    float x = 0, y = 0, z = 0;
};
struct Tag {
    int value = 0;
};
struct Heavy {
    double a[8] = {};
};
}  // namespace

TEST(destroyed_entity_handles_do_not_alias_recycled_slots) {
    World w;
    Entity a = w.create();
    w.add<Tag>(a, Tag{7});
    w.destroy(a);
    Entity b = w.create();
    CHECK_EQ(entityIndex(a), entityIndex(b));
    CHECK(!w.alive(a));
    CHECK(w.alive(b));
    CHECK(w.tryGet<Tag>(a) == nullptr);
    CHECK(w.tryGet<Tag>(b) == nullptr);
}

TEST(swap_remove_keeps_every_other_component_reachable) {
    World w;
    std::vector<Entity> entities;
    std::unordered_map<Entity, int> expected;
    for (int i = 0; i < 500; ++i) {
        Entity e = w.create();
        w.add<Tag>(e, Tag{i});
        entities.push_back(e);
        expected[e] = i;
    }

    std::mt19937 rng(5);
    std::shuffle(entities.begin(), entities.end(), rng);
    for (size_t i = 0; i < entities.size(); i += 3) {
        w.remove<Tag>(entities[i]);
        expected.erase(entities[i]);
    }

    CHECK_EQ(w.pool<Tag>().size(), expected.size());
    for (const auto& [e, value] : expected) {
        const Tag* t = w.tryGet<Tag>(e);
        CHECK(t != nullptr);
        CHECK_EQ(t->value, value);
    }
    for (size_t i = 0; i < w.pool<Tag>().dense.size(); ++i) {
        Entity owner = w.pool<Tag>().dense[i];
        CHECK_EQ(w.pool<Tag>().data[i].value, expected.at(owner));
    }
}

TEST(destroy_clears_components_from_every_pool) {
    World w;
    Entity e = w.create();
    w.add<Pos>(e, Pos{1, 2, 3});
    w.add<Tag>(e, Tag{9});
    w.add<Heavy>(e);
    CHECK_EQ(w.pool<Pos>().size(), size_t{1});
    w.destroy(e);
    CHECK_EQ(w.pool<Pos>().size(), size_t{0});
    CHECK_EQ(w.pool<Tag>().size(), size_t{0});
    CHECK_EQ(w.pool<Heavy>().size(), size_t{0});
    CHECK_EQ(w.aliveCount(), size_t{0});
}

TEST(multi_component_iteration_visits_exactly_the_intersection) {
    World w;
    std::mt19937 rng(17);
    std::unordered_set<Entity> both;
    for (int i = 0; i < 2000; ++i) {
        Entity e = w.create();
        bool hasPos = (rng() % 3) != 0;
        bool hasTag = (rng() % 2) == 0;
        if (hasPos) w.add<Pos>(e, Pos{static_cast<float>(i), 0, 0});
        if (hasTag) w.add<Tag>(e, Tag{i});
        if (hasPos && hasTag) both.insert(e);
    }

    std::unordered_set<Entity> seen;
    forEach<Tag, Pos>(w, [&](Entity e, Tag& t, Pos& p) {
        CHECK_EQ(static_cast<int>(p.x), t.value);
        CHECK(seen.insert(e).second);
    });
    CHECK_EQ(seen.size(), both.size());
    for (Entity e : both) CHECK(seen.count(e) == 1);
}

TEST(iteration_is_safe_while_removing_the_visited_component) {
    World w;
    for (int i = 0; i < 1000; ++i) {
        Entity e = w.create();
        w.add<Tag>(e, Tag{i});
    }
    std::vector<Entity> doomed;
    forEach<Tag>(w, [&](Entity e, Tag& t) {
        if (t.value % 2 == 0) doomed.push_back(e);
    });
    for (Entity e : doomed) w.remove<Tag>(e);
    CHECK_EQ(w.pool<Tag>().size(), size_t{500});
    forEach<Tag>(w, [&](Entity, Tag& t) { CHECK(t.value % 2 == 1); });
}

TEST(reinserting_a_component_overwrites_instead_of_duplicating) {
    World w;
    Entity e = w.create();
    w.add<Tag>(e, Tag{1});
    w.add<Tag>(e, Tag{2});
    CHECK_EQ(w.pool<Tag>().size(), size_t{1});
    CHECK_EQ(w.get<Tag>(e).value, 2);
}

TEST(entity_slots_are_recycled_under_churn) {
    World w;
    std::vector<Entity> live;
    std::mt19937 rng(29);
    for (int round = 0; round < 200; ++round) {
        for (int i = 0; i < 20; ++i) {
            Entity e = w.create();
            w.add<Pos>(e, Pos{});
            live.push_back(e);
        }
        for (int i = 0; i < 15 && !live.empty(); ++i) {
            size_t idx = rng() % live.size();
            w.destroy(live[idx]);
            live[idx] = live.back();
            live.pop_back();
        }
    }
    CHECK_EQ(w.aliveCount(), live.size());
    CHECK_EQ(w.pool<Pos>().size(), live.size());
    CHECK(w.capacity() < live.size() + 100);
    for (Entity e : live) CHECK(w.alive(e));
}
